# Quickstart

:::{note}
Planned for M3. This page will walk through the controller demos: `loopback_demo` on Jazzy and
later, and `kinematic_sim_demo` on every supported distro, both opening the pre-configured RViz
view. The `follow_curve` walkthrough with `curve_follower_demo` and `send_curve.py` follows in M5.
:::

Until then, the repository builds from source in a standard colcon workspace:

```bash
mkdir -p ~/vf_ws/src && cd ~/vf_ws/src
git clone https://github.com/h3ct0r/vectorfield_nav2.git
cd ~/vf_ws
rosdep install --from-paths src --ignore-src -y
colcon build
```
