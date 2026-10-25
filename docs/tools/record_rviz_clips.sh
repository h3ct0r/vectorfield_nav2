#!/usr/bin/env bash
# Copyright 2026 Héctor Azpúrua
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Record the phase clips of the visualization guide (docs/visualization.md, "Screenshots").
#
# Each phase runs kinematic_sim_demo (differential drive, open_10x10) with RViz on the shipped
# rviz/vectorfield.rviz under Xvfb, with only the camera changed (top-down) and the field grid
# off; ffmpeg films the screen while the goals of the phase run. From each recording it writes
#
#   docs/_static/media/viz_<clip>.mp4   the run, cropped, at most 960 px wide (H.264, CRF 28)
#   docs/_static/media/viz_<clip>.gif   the same at 480 px, 10 fps
#   docs/_static/screenshots/<clip>.png the clip's poster: one frame of it
#
# Usage, in the Jazzy dev container with the workspace built and sourced, and Xvfb and ffmpeg
# installed (apt install xvfb ffmpeg):
#
#   docs/tools/record_rviz_clips.sh [--work DIR] [--encode-only] [phase ...]
#
#   phase          follow, rotate_to_field, final_approach (which also records
#                  rotate_to_heading), contour, blocked (default: all)
#   --work         keep the raw recordings, their goal times and the logs there (default: a
#                  temporary directory)
#   --encode-only  encode the recordings of --work again without running the phases
#
# Environment: VF_CLIPS_TIMEOUT (s per goal, default 90).

set -euo pipefail

PHASES=(follow rotate_to_field final_approach contour blocked)
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TOOLS="$REPO/docs/tools"
MEDIA="$REPO/docs/_static/media"
SHOTS="$REPO/docs/_static/screenshots"
WORK=""
ENCODE_ONLY=0
TIMEOUT="${VF_CLIPS_TIMEOUT:-90}"
SCREEN=2400x1400
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1

selected=()
while [ $# -gt 0 ]; do
  case "$1" in
    --work) WORK="$2"; shift 2 ;;
    --encode-only) ENCODE_ONLY=1; shift ;;
    -h | --help)
      sed -n '/^# Record/,/^# Environment/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    -*) echo "record_rviz_clips: unknown option $1" >&2; exit 2 ;;
    *) selected+=("$1"); shift ;;
  esac
done
[ ${#selected[@]} -gt 0 ] || selected=("${PHASES[@]}")
tools=(ffmpeg)
[ "$ENCODE_ONLY" = 1 ] || tools+=(Xvfb rviz2)
for tool in "${tools[@]}"; do
  command -v "$tool" >/dev/null || { echo "record_rviz_clips: $tool not found" >&2; exit 1; }
done
if [ -z "$WORK" ]; then
  [ "$ENCODE_ONLY" = 0 ] || { echo 'record_rviz_clips: --encode-only needs --work' >&2; exit 2; }
  WORK="$(mktemp -d)"
  trap 'rm -rf "$WORK"' EXIT
fi
mkdir -p "$WORK" "$MEDIA" "$SHOTS"

if [ "$ENCODE_ONLY" = 0 ] && ! pgrep -x Xvfb >/dev/null; then
  Xvfb :99 -screen 0 "${SCREEN}x24" -nolisten tcp >"$WORK/xvfb.log" 2>&1 &
  sleep 2
fi

GROUPS_RUNNING=()
# Start a command in its own process group, so that stop_all reaches its children.
start() {
  local log="$1"; shift
  setsid "$@" >"$log" 2>&1 &
  GROUPS_RUNNING+=("$!")
}
stop_all() {
  local pid
  for pid in "${GROUPS_RUNNING[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 5
  for pid in "${GROUPS_RUNNING[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  GROUPS_RUNNING=()
  sleep 1
}
trap 'stop_all' INT TERM

# The demo and RViz with a view-only config: stop_all ends both.
demo() {
  local name="$1" view="$2"; shift 2
  # shellcheck disable=SC2086  # the view arguments are words
  python3 "$TOOLS/rviz_clips.py" view "$WORK/$name.rviz" $view
  start "$WORK/$name.launch.log" ros2 launch vectorfield_bringup kinematic_sim_demo.launch.py \
    use_rviz:=false "$@"
  sleep 3
  start "$WORK/$name.rviz.log" rviz2 --fullscreen -d "$WORK/$name.rviz" \
    --ros-args -p use_sim_time:=true
  local i
  for i in $(seq 1 90); do
    if ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active &&
      ros2 lifecycle get /controller_server 2>/dev/null | grep -q active; then
      sleep 4
      return 0
    fi
    sleep 2
  done
  echo "record_rviz_clips: $name: Nav2 did not come up; see $WORK/$name.launch.log" >&2
  return 1
}

# Film the screen losslessly until stop_film; mark records a goal's start and end in seconds of
# the film, in <work>/<phase>.times as "<clip> <start> <end>" lines.
film() {
  FILM="$WORK/$1.mkv"
  TIMES="$WORK/$1.times"
  : >"$TIMES"
  ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 10 -video_size "$SCREEN" \
    -i :99.0 -c:v libx264rgb -crf 0 -preset ultrafast "$FILM" &
  FILM_PID=$!
  FILM_T0=$(date +%s.%N)
  sleep 0.5
}
stop_film() {
  kill -INT "$FILM_PID" 2>/dev/null || true
  wait "$FILM_PID" 2>/dev/null || true
}
since() { awk -v a="$FILM_T0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.2f", b - a }'; }

# goal CLIP X Y YAW_DEG [WAIT]: one NavigateToPose goal, its times marked as CLIP.
goal() {
  local clip="$1" wait="${5:-$TIMEOUT}" start
  start=$(since)
  timeout $((wait + 30)) python3 "$TOOLS/rviz_clips.py" goal "$2" "$3" "$4" --wait "$wait" || true
  echo "$clip $start $(since)" >>"$TIMES"
}

# Each clip: LEAD (s after the goal is sent: Nav2 plans first), the most seconds kept, the crop
# W:H:X:Y of the 2400x1400 screen and the poster time (s into the clip).
clip_settings() {
  case "$1" in
    follow) LEAD=1.5 KEEP=16 CROP=1400:500:500:500 POSTER=4.5 ;;
    rotate_to_field) LEAD=1.5 KEEP=16 CROP=860:800:640:440 POSTER=1 ;;
    final_approach) LEAD=1.5 KEEP=12 CROP=1100:440:400:430 POSTER=5 ;;
    rotate_to_heading) LEAD=1 KEEP=6 CROP=700:520:930:440 POSTER=0.8 ;;
    contour) LEAD=1.5 KEEP=24 CROP=1400:600:500:400 POSTER=2.5 ;;
    blocked) LEAD=1.5 KEEP=9 CROP=1200:560:620:420 POSTER=7 ;;
  esac
}

# encode PHASE: viz_<clip>.mp4/.gif and the PNG poster of every clip marked in the phase's film.
encode() {
  local film="$WORK/$1.mkv" clip start end length gif
  while read -r clip start end; do
    clip_settings "$clip"
    start=$(awk -v a="$start" -v b="$LEAD" 'BEGIN { print a + b }')
    length=$(awk -v a="$start" -v b="$end" -v k="$KEEP" \
      'BEGIN { l = b - a + 1; print (l < k) ? l : k }')
    echo "== $clip: ${length} s from ${start} s, crop $CROP"
    ffmpeg -hide_banner -loglevel error -y -ss "$start" -t "$length" -i "$film" -an \
      -vf "crop=$CROP,scale='min(960,iw)':-2:flags=lanczos,format=yuv420p" \
      -c:v libx264 -preset slow -crf 28 -movflags +faststart "$MEDIA/viz_$clip.mp4"
    gif="crop=$CROP,fps=10,scale=480:-2:flags=lanczos,split[a][b]"
    gif+=";[a]palettegen=max_colors=128:stats_mode=diff[p]"
    gif+=";[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle"
    ffmpeg -hide_banner -loglevel error -y -ss "$start" -t "$length" -i "$film" -an \
      -filter_complex "[0:v]$gif" -loop 0 "$MEDIA/viz_$clip.gif"
    ffmpeg -hide_banner -loglevel error -y \
      -ss "$(awk -v a="$start" -v b="$POSTER" 'BEGIN { print a + b }')" -i "$film" \
      -frames:v 1 -vf "crop=$CROP" "$SHOTS/$clip.png"
  done <"$WORK/$1.times"
}

# FOLLOW: goal (4, 0) from the origin, and a 0.45 m localization jump to the side at x = 1.
phase_follow() {
  demo follow '--scale 200 --x 1.6 --y 0.8'
  film follow
  python3 "$TOOLS/rviz_clips.py" step 1.0 0 0.45 0 >"$WORK/follow.step.log" 2>&1 &
  goal follow 4 0 0
}

# ROTATE_TO_FIELD: goal (-2.5, -2.5) behind the robot, which faces +x at the origin.
phase_rotate_to_field() {
  demo rotate_to_field '--scale 200 --x -0.6 --y -0.3'
  film rotate_to_field
  goal rotate_to_field -2.5 -2.5 0
}

# FINAL_APPROACH, then ROTATE_TO_HEADING: start (1.8, -1.8) facing the origin, the goal at the
# origin; then a second goal there with the yaw 90 degrees to the right. The view is turned by
# 135 degrees, so the robot drives to the right.
phase_final_approach() {
  demo final_approach '--scale 350 --angle 2.35619 --x 0.25 --y -0.25' \
    initial_pose_x:=1.8 initial_pose_y:=-1.8 initial_pose_yaw:=2.35619
  film final_approach
  goal final_approach 0 0 135
  sleep 1
  goal rotate_to_heading 0 0 45
}

# Contour episode: demo_obstacles, start (1.6, -1.6) facing the goal (-1.6, 1.6), and a 0.4 m box
# set at run time on the plan at the origin. The view is turned by 135 degrees.
phase_contour() {
  demo contour '--scale 220 --angle 2.35619 --x 0 --y 0' scenarios:=demo_obstacles \
    initial_pose_x:=1.6 initial_pose_y:=-1.6 initial_pose_yaw:=2.35619
  ros2 param set /vectorfield_sim box_obstacles \
    "[2.0, 0.0, 0.4, 0.4, 0.0, -2.0, 2.0, 1.5, 0.2, 0.785, 0.0, 0.0, 0.4, 0.4, 0.785]"
  sleep 3
  film contour
  goal contour -1.6 1.6 135
}

# BLOCKED with obstacle_mode none: goal (4, 0) from the origin, an unmapped 13 m wall at x = 2.6
# across the map, and a global costmap without the obstacle layer, so the plan runs through it.
phase_blocked() {
  cat >"$WORK/blocked_none.yaml" <<'EOF'
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "inflation_layer"]
controller_server:
  ros__parameters:
    FollowPath:
      obstacle_mode: "none"
EOF
  demo blocked '--scale 260 --x 1.9 --y 0.4' scenarios:="$WORK/blocked_none.yaml"
  ros2 param set /vectorfield_sim box_obstacles "[2.6, 0.0, 0.2, 13.0, 0.0]"
  sleep 3
  film blocked
  goal blocked 4 0 0 14
}

for phase in "${selected[@]}"; do
  case "$phase" in
    follow | rotate_to_field | final_approach | contour | blocked) ;;
    *) echo "record_rviz_clips: unknown phase '$phase' (one of: ${PHASES[*]})" >&2; exit 2 ;;
  esac
  if [ "$ENCODE_ONLY" = 0 ]; then
    "phase_$phase"
    stop_film
    stop_all
  fi
  encode "$phase"
done
echo '== sizes'
ls -l "$MEDIA"/viz_* "$SHOTS"/*.png | awk '{ printf "%8d KiB  %s\n", ($5 + 1023) / 1024, $NF }'
