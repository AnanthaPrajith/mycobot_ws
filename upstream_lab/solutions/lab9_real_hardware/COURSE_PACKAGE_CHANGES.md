# Course packages vs. our Lab 9 packages (2026-10-05)

Written 2026-10-05 15:45. **The provided course packages in `pp_moveit_ws/src/mycobot_*` are unchanged:**
`git diff 04c715b -- pp_moveit_ws/src/mycobot_*` is empty. All Lab 9 code lives in two own packages
(`lab9_interfaces`, `lab9_pick_place`) that use the course packages without modifying them.

## History of the course packages

| Commit | Time | Course packages |
|---|---|---|
| `2218733`, `3a77aab` | 09-28 | imported as delivered (`3a77aab` only added the full `mycobot_description`) |
| `04c715b` | 10-05 11:14 | **as delivered** (reference) |
| `2b1f0bf`, `c456522` | 10-05 13:40, 15:13 | temporarily modified (2 files) and extended (4 files, 5 small config edits) |
| `48c54ce` | 10-05 15:37 | **restored to `04c715b`**; everything moved into `lab9_interfaces` / `lab9_pick_place` |

## What replaces what

| Former change inside a course package | Now | How it is used instead |
|---|---|---|
| `mycobot_controller/controller.py`: valve GPIO 21 + smooth execution | `lab9_pick_place/controller_lab9.py` (copy + both fixes) | Pi: `python3 ~/mycobot-project/pp_moveit_ws/src/lab9_pick_place/lab9_pick_place/controller_lab9.py` **instead of** `ros2 run mycobot_controller controller` (not built on the Pi). Same node name, topics and action, so MoveIt and both brains work with either |
| `mycobot_280_moveit2/launch/move_group.launch.py`: execution limits × 4 + 5 s | `lab9_pick_place/launch/lab9_moveit.launch.py` (same MoveIt config + 3 parameters, incl. the course's start tolerance 0.3) | `ros2 launch lab9_pick_place lab9_moveit.launch.py` instead of the course launch; `lab9_real.launch.py` uses it |
| `mycobot_interfaces/srv/GetObject.srv` + `CMakeLists.txt` line | `lab9_interfaces/srv/GetObject.srv` | type `lab9_interfaces/srv/GetObject` (service name `/get_object` unchanged) |
| `mycobot_vision/vision_lab9.py` + `setup.py` entry + `package.xml` yaml | `lab9_pick_place/vision_lab9.py` | `ros2 run lab9_pick_place vision_lab9` (still serves the course `/cube_coordinates` too) |
| `mycobot_brain/brain_lab9.py` + `setup.py` entry + `package.xml` yaml | `lab9_pick_place/brain_lab9.py` | `ros2 run lab9_pick_place brain_lab9` |
| `mycobot_brain/launch/lab9_real.launch.py` | `lab9_pick_place/launch/lab9_real.launch.py` | `ros2 launch lab9_pick_place lab9_real.launch.py` |
| `solutions/lab9_real_hardware/lab9.yaml` | `lab9_pick_place/config/lab9.yaml` | default parameter `config` of vision + brain (installed as a symlink: edit, restart, no rebuild) |

What we use from the course, unchanged: MoveIt config and URDF scene (`mycobot_280_moveit2`, `mycobot_description`),
camera node (`mycobot_280pi opencv_camera`), `GetCubeCoords` (`mycobot_interfaces`).

## Part 0 baseline

The original system now runs exactly as delivered: [Pi] `ros2 run mycobot_controller controller`, Lab PC
`ros2 launch mycobot_brain brain.launch.py`. Expected on the real robot (seen on 10-05 with the same code): slow
stop-and-go motion, `TIMED_OUT` after most executions while the arm keeps moving (the controller ignores cancel), object
may stay on the pump (pump off without the release valve). These are findings for the Part 0 report; the fixes are in
`controller_lab9` and `lab9_moveit.launch.py`.

## Rebuild after switching (done on the Lab PC 10-05 15:40)

The old builds of `mycobot_interfaces`, `mycobot_vision`, `mycobot_brain` contained the moved files, so their `build/` and
`install/` folders were deleted and rebuilt from the original sources:
```bash
cd ~/mycobot-project/pp_moveit_ws && source /opt/ros/jazzy/setup.bash
rm -rf build/mycobot_interfaces install/mycobot_interfaces build/mycobot_vision install/mycobot_vision build/mycobot_brain install/mycobot_brain
colcon build --symlink-install --packages-select mycobot_interfaces mycobot_vision mycobot_brain lab9_interfaces lab9_pick_place
```
On the Pi only `mycobot_controller` was built before; it imports from `src/` (develop install), so after `git pull` it is
the original again without a rebuild. `lab9_pick_place` cannot be built on the Pi (its `package.xml` depends on course
packages that are not built there); `controller_lab9.py` is started directly with `python3` instead.
