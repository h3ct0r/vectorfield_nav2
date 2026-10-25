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

# Record the gallery clips of the README and the docs' Examples pages.
#
# Each MuJoCo example runs headless to the end of its scenario, filmed in simulated time as its
# scenario's `recording` says (camera, view, hidden markers, the robot's trail):
#
#   ros2 launch vectorfield_mujoco <example>.launch.py use_rviz:=false exit_on_done:=true \
#       record_path:=<work>/<example>.raw.mp4
#
# and ffmpeg turns the recording into docs/_static/media/<example>.{mp4,gif,jpg}:
#
#   mp4  960x540 H.264 (yuv420p, +faststart, CRF 28), the whole run from the first motion, sped
#        up for the long runs, at most 3 MiB;
#   gif  480 px wide, 12 fps, the highlight of the run, at most 2 MiB;
#   jpg  a poster frame, at most 150 KiB.
#
# A caption in the corner names the mode, the robot and the driver, and the speed-up if any.
# The script fails when a scenario does not succeed or a file exceeds its budget.
#
# Usage, in the dev container with the workspace built and sourced (ffmpeg, an OpenGL backend
# such as OSMesa, and the DejaVu fonts installed):
#
#   vectorfield_mujoco/scripts/record_gallery.sh [--out DIR] [--work DIR] [--encode-only] \
#       [example ...]
#
#   example        differential, skid_steer, omnidirectional, ackermann, drone,
#                  curve_following (default: all six)
#   --out DIR      where the media go (default: docs/_static/media of this repository)
#   --work DIR     keep the raw recordings and the launch logs there (default: a temporary
#                  directory, removed at the end unless an example failed)
#   --encode-only  encode <work>/<example>.raw.mp4 again without running the example
#
# Environment: VF_GALLERY_TIMEOUT (s per example, default 1500), VF_GALLERY_FONT (a TTF file).

set -euo pipefail

EXAMPLES=(differential skid_steer omnidirectional ackermann drone curve_following)
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$REPO/docs/_static/media"
WORK=""
ENCODE_ONLY=0
TIMEOUT="${VF_GALLERY_TIMEOUT:-1500}"

# Size budgets (bytes): pre-commit's check-added-large-files allows 3072 KiB.
MP4_BUDGET=$((3 * 1024 * 1024))
GIF_BUDGET=$((2 * 1024 * 1024))
JPG_BUDGET=$((150 * 1024))

# Per example: the caption; the MP4's speed-up; the GIF's highlight: its start (simulated s
# after the first motion), its length (s of GIF) and its speed-up; and the poster's time
# (simulated s after the first motion). GIF_COLORS (default 128) is the GIF's palette size: a
# moving camera changes every pixel of every frame and needs a smaller one to fit the budget.
settings() {
  GIF_COLORS=128
  case "$1" in
    differential)
      # Contour around the box, rejoin, rotate at the goal.
      CAPTION='Differential · PAL TIAGo · Nav2 + obstacle contour'
      SPEED=2 GIF_FROM=0 GIF_LEN=10 GIF_SPEED=2 POSTER=12
      ;;
    skid_steer)
      # Through the door to the end of the corridor, turn around.
      CAPTION='Skid-steer · AgileX Scout Mini · Nav2 through a door'
      SPEED=2 GIF_FROM=0 GIF_LEN=10 GIF_SPEED=2 POSTER=11.5 GIF_COLORS=64
      ;;
    omnidirectional)
      # Most of a lap, strafing at a fixed heading.
      CAPTION='Omnidirectional · TidyBot base · curve follower, fixed heading'
      SPEED=2 GIF_FROM=6 GIF_LEN=12 GIF_SPEED=2 POSTER=28.5
      ;;
    ackermann)
      # The swerve around the box, back to the path, to the goal.
      CAPTION='Ackermann · car model · Nav2 Smac Hybrid + CBF'
      SPEED=1 GIF_FROM=0 GIF_LEN=11 GIF_SPEED=1 POSTER=7.5
      ;;
    drone)
      # Take-off and the helix around the pillar and the sphere.
      CAPTION='Drone · Skydio X2 · 3D follower helix + CBF'
      SPEED=1.5 GIF_FROM=0 GIF_LEN=10 GIF_SPEED=2 POSTER=19 GIF_COLORS=64
      ;;
    curve_following)
      # About a lap of the lemniscate, through its crossing.
      CAPTION='Curve following · AgileX Scout Mini · lemniscate laps'
      SPEED=2 GIF_FROM=7 GIF_LEN=13 GIF_SPEED=2 POSTER=18.5
      ;;
    *)
      echo "record_gallery: unknown example '$1' (one of: ${EXAMPLES[*]})" >&2
      return 1
      ;;
  esac
}

usage() {
  sed -n '/^# Usage/,/^# Environment/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

selected=()
while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --work) WORK="$2"; shift 2 ;;
    --encode-only) ENCODE_ONLY=1; shift ;;
    -h | --help) usage; exit 0 ;;
    -*) echo "record_gallery: unknown option $1" >&2; usage >&2; exit 2 ;;
    *) settings "$1" >/dev/null; selected+=("$1"); shift ;;
  esac
done
[ ${#selected[@]} -gt 0 ] || selected=("${EXAMPLES[@]}")

for tool in ffmpeg ffprobe; do
  command -v "$tool" >/dev/null || {
    echo "record_gallery: $tool not found (apt install ffmpeg)" >&2
    exit 1
  }
done
FONT="${VF_GALLERY_FONT:-/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf}"
if [ ! -f "$FONT" ]; then
  FONT="$(fc-match -f '%{file}' 'DejaVu Sans' 2>/dev/null || true)"
fi
[ -f "$FONT" ] || {
  echo 'record_gallery: no DejaVu Sans (apt install fonts-dejavu-core)' >&2
  exit 1
}
if [ "$ENCODE_ONLY" = 0 ]; then
  ros2 pkg prefix vectorfield_mujoco >/dev/null 2>&1 || {
    echo 'record_gallery: vectorfield_mujoco not found: build the workspace and source it' >&2
    exit 1
  }
fi
if [ -z "$WORK" ]; then
  [ "$ENCODE_ONLY" = 0 ] || { echo 'record_gallery: --encode-only needs --work' >&2; exit 2; }
  WORK="$(mktemp -d)"
  # Kept when an example failed: its log and raw recording are what to look at.
  cleanup() {
    if [ "${failed:-1}" = 0 ]; then
      rm -rf "$WORK"
    else
      echo "record_gallery: kept $WORK" >&2
    fi
  }
  trap cleanup EXIT
fi
LAUNCH=''

# Ctrl-C or SIGTERM: stop the running example first (timeout runs it in its own process group,
# out of reach of the terminal's signals), then exit; the EXIT trap cleans up after it.
interrupt() {
  if [ -n "$LAUNCH" ]; then
    kill -INT "$LAUNCH" 2>/dev/null || true
    wait "$LAUNCH" 2>/dev/null || true
  fi
  exit "$1"
}
trap 'interrupt 130' INT
trap 'interrupt 143' TERM
mkdir -p "$OUT" "$WORK"

# ffmpeg 6.1's drawtext (Ubuntu Noble's) drops one character from the end of a text per extra
# byte of its UTF-8 characters (the captions' "·"): pad such texts with as many spaces.
PAD_UTF8=0
case "$(ffmpeg -hide_banner -version | head -n 1)" in
  'ffmpeg version 6.1'* | 'ffmpeg version n6.1'*) PAD_UTF8=1 ;;
esac

# Write the caption text $1 to the file $2.
write_caption() {
  local pad=0
  if [ "$PAD_UTF8" = 1 ]; then
    pad=$(python3 -c 'import sys; t = sys.argv[1]; print(len(t.encode()) - len(t))' "$1")
  fi
  printf '%s%*s' "$1" "$pad" '' >"$2"
}

calc() { awk "BEGIN { printf \"%.3f\", $* }"; }
round() { awk "BEGIN { printf \"%d\", $* + 0.5 }"; }
duration() { ffprobe -v error -show_entries format=duration -of csv=p=0 "$1"; }
kib() { echo $((($(wc -c <"$1") + 1023) / 1024)); }

# The drawtext filter of the caption file $1 in the bottom left corner, $2 pixels high.
caption() {
  local margin
  margin=$(round "$2 * 0.7")
  printf "drawtext=fontfile='%s':textfile='%s':expansion=none:fontsize=%s:fontcolor=white" \
    "$FONT" "$1" "$2"
  printf ':box=1:boxcolor=black@0.55:boxborderw=%s:x=%s:y=h-th-%s' \
    "$(round "$2 * 0.35")" "$margin" "$margin"
}

# Print "<first motion> <last motion>" (s) of a recording: the ends of its still head and tail
# (ffmpeg's freezedetect), or 0 and its duration. The head may hold blips shorter than 0.6 s (a
# robot settling on its wheels at the start).
motion() {
  ffmpeg -hide_banner -nostats -i "$1" -an -vf 'freezedetect=n=0.0005:d=0.5' -f null - 2>&1 |
    awk -v total="$(duration "$1")" '
      /freeze_start:/ { start = $NF; open = 1 }
      /freeze_end:/ { if (!moved && start - head <= 0.6) head = $NF; else moved = 1; open = 0 }
      END { printf "%s %s\n", head + 0, (open && start > head) ? start : total }'
}

# Run an example to the end of its scenario, recording to $2; its log goes to $3.
record() {
  local ex="$1" raw="$2" log="$3" status=0
  echo "== $ex: running the example (log: $log)"
  rm -f "$raw"
  # In the background so that the traps above run while it does (bash defers them until a
  # foreground command ends).
  timeout -s INT -k 60 "$TIMEOUT" ros2 launch vectorfield_mujoco "$ex.launch.py" \
    use_rviz:=false exit_on_done:=true record_path:="$raw" >"$log" 2>&1 &
  LAUNCH=$!
  wait "$LAUNCH" || status=$?
  LAUNCH=''
  if ! grep -q 'Scenario succeeded' "$log"; then
    echo "record_gallery: $ex: the scenario did not succeed (exit $status); see $log" >&2
    grep -E 'scenario [a-z_]+: |ERROR|Traceback' "$log" | tail -n 5 >&2 || true
    return 1
  fi
  if [ ! -s "$raw" ] || ! grep -q 'recorded [0-9]* frames' "$log"; then
    echo "record_gallery: $ex: no recording; see $log" >&2
    return 1
  fi
}

# Encode the media of an example from its raw recording.
encode() {
  local ex="$1" raw="$2" first last start length suffix gif_suffix poster
  settings "$ex"
  read -r first last < <(motion "$raw")
  # Half a second of stillness before the first motion, one after the last.
  start=$(calc "($first > 0.5) ? $first - 0.5 : 0")
  length=$(calc "$last + 1 - $start")
  suffix=''
  [ "$SPEED" = 1 ] || suffix=" · ${SPEED}x"
  gif_suffix=''
  [ "$GIF_SPEED" = 1 ] || gif_suffix=" · ${GIF_SPEED}x"
  write_caption "$CAPTION$suffix" "$WORK/$ex.caption.txt"
  write_caption "$CAPTION$gif_suffix" "$WORK/$ex.gif-caption.txt"
  echo "== $ex: motion from ${first} s to ${last} s; encoding"

  ffmpeg -hide_banner -loglevel error -y -ss "$start" -t "$length" -i "$raw" -an \
    -vf "setpts=(PTS-STARTPTS)/$SPEED,fps=30,scale=960:540,$(caption "$WORK/$ex.caption.txt" 20)" \
    -c:v libx264 -preset slow -crf 28 -pix_fmt yuv420p -movflags +faststart "$OUT/$ex.mp4" ||
    return 1

  # One palette for the whole GIF; ordered dithering and changed rectangles keep it small.
  local gif="setpts=(PTS-STARTPTS)/$GIF_SPEED,fps=12,scale=480:-2:flags=lanczos"
  gif+=",$(caption "$WORK/$ex.gif-caption.txt" 12),split[a][b]"
  gif+=";[a]palettegen=max_colors=$GIF_COLORS:stats_mode=diff[p]"
  gif+=";[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle"
  ffmpeg -hide_banner -loglevel error -y -ss "$(calc "$first + $GIF_FROM")" \
    -t "$(calc "$GIF_LEN * $GIF_SPEED")" -i "$raw" -an -filter_complex "[0:v]$gif" \
    -loop 0 "$OUT/$ex.gif" || return 1

  # The poster, at full size, with the MP4's caption.
  poster=$(calc "$first + $POSTER")
  ffmpeg -hide_banner -loglevel error -y -ss "$poster" -i "$raw" -an -frames:v 1 \
    -vf "scale=960:540,$(caption "$WORK/$ex.caption.txt" 20)" -q:v 5 "$OUT/$ex.jpg" || return 1
}

failed=0
report=()
for ex in "${selected[@]}"; do
  raw="$WORK/$ex.raw.mp4"
  if [ "$ENCODE_ONLY" = 0 ]; then
    record "$ex" "$raw" "$WORK/$ex.log" || { failed=1; continue; }
  elif [ ! -s "$raw" ]; then
    echo "record_gallery: $ex: no $raw to encode" >&2
    failed=1
    continue
  fi
  encode "$ex" "$raw"
  line="$ex:"
  for kind in mp4 gif jpg; do
    file="$OUT/$name.$kind"
    case "$kind" in
      mp4) budget=$MP4_BUDGET ;;
      gif) budget=$GIF_BUDGET ;;
      jpg) budget=$JPG_BUDGET ;;
    esac
    size=$(wc -c <"$file")
    line+=" $kind $(kib "$file") KiB"
    [ "$kind" = jpg ] || line+=" ($(printf '%.1f' "$(duration "$file")") s)"
    if [ "$size" -gt "$budget" ]; then
      echo "record_gallery: $file is $(kib "$file") KiB, over $((budget / 1024)) KiB" >&2
      status=1
    fi
  done
  report+=("$line")
done

echo '== gallery sizes'
for line in "${report[@]}"; do echo "$line"; done
shopt -s nullglob
media=("$OUT"/*.mp4 "$OUT"/*.gif "$OUT"/*.jpg)
shopt -u nullglob
total=0
[ ${#media[@]} -eq 0 ] || total=$(cat "${media[@]}" | wc -c)
echo "total in $OUT: $(((total + 1023) / 1024)) KiB"
exit "$failed"
