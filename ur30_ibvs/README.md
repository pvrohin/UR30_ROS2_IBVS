# ur30_ibvs

Image-based visual servoing (ViSP) of the eye-in-hand camera onto the door panel.
Stages: survey -> detect panel -> approach one long edge -> servo on that edge.
On top of that there is a dispense lap demo (below).

## Dispense lap demo: lateral tracking while the part moves

The camera is taken once round the whole boundary of the panel (a stand-in for dispensing
along the part), while the panel is nudged at random. IBVS keeps the camera on the edge.

```bash
ros2 launch ur30_ibvs ur30_ibvs.launch.py demo:=true
# options: shifts:=false (clean lap, panel never moves), shift_seed:=<n> (repeat a run),
#          gazebo_gui:=false launch_rviz:=false (headless)
```

What happens:

1. As in the normal run: survey, panel found, approach to 0.2 m above one long edge.
2. `DISPENSE`: the camera slides along the edge at `dispense_speed` (0.03 m/s, the
   "pre-programmed" motion) counter-clockwise round the panel: half of the starting
   edge, three whole edges, the other half. IBVS (vpMeLine + vpServo) holds the edge on the
   image centre row and parallel to the image axis (across-edge translation and yaw), and
   the standoff comes from TF.
3. The heading stays fixed. On each side the image is turned by a multiple of 90 degrees
   (`virtual_view.hpp`, a "virtual camera") so the edge looks horizontal and the travel goes
   to the image right, which is what the controller expects.
4. The corner is found in that image (`corner_finder.hpp`: where the dark part ends in a
   strip just inside the tracked edge). When it is at the image centre the next side
   starts, the tracker is re-seeded on it.
5. `part_shifter.py` waits for `DISPENSE`, then every 4-8 s moves the panel (a separate
   Gazebo model) to a random pose within 2 cm and 2 degrees of where it started, over 1 s.
   It scores the distance of the point under the camera's optical axis from the panel's
   true boundary, and logs a PASS/FAIL line at the end (`results_file:=` writes JSON).
   Pass: lap reached `DONE` and the error, once the panel has been still for 2 s, stayed
   within 8 mm.

RViz (`ur30_cell_bringup`) shows the panel at the start (red), where it is now (green) and
the camera's ground trail (yellow).

### Results so far (headless Gazebo, real-time factor about 0.8)

| Run | Nudges | Lap | Settled error max | Error during nudges max |
|---|---|---|---|---|
| clean lap (`shifts:=false`) | 0 | 108 s, 4 corners | 6.0 mm (rms 1.0 mm) | n/a |
| `shift_seed:=1` | 16 | 103 s, 4 corners | 5.4 mm | 43.6 mm (rms 5.0 mm) |
| `shift_seed:=2` | 14 | 103 s, 4 corners | 2.9 mm | 36.0 mm (rms 4.0 mm) |
| `shift_seed:=3` | 14 | 104 s, 4 corners | 6.7 mm | 31.7 mm (rms 3.5 mm) |
| `shift_seed:=4` | 15 | 105 s, 4 corners | 4.0 mm | 35.6 mm (rms 4.1 mm) |

Seeds 1 and 3 ran before a corner-finder fix (a bright stretch at the start of the strip
was read as a corner: seed 2 failed on it, then passed after the fix). Seeds 2 and 4 are
with the final code; 1 and 3 have not been re-run.

The edge was lost and re-acquired on the larger nudges (4 cm between consecutive poses).

### Things to know

- **The demo turns the panel by 90 degrees** (`ur30_ibvs_demo.yaml`, and the spawn yaw in the
  launch file). With the 0.9 m side along world x, the far end of the lap is not reachable
  with the camera pointing down 1.0 m above the table: Servo stopped with "Very close to a
  singularity". Turned, the farthest point is 1.17 m from the base (offline kinematics:
  Jacobian condition number at most ~16 along the lap). The survey pose is turned with it
  (wrist_3 -90 degrees).
- The tracked point is the camera's optical axis, not a dispense needle. A constant needle
  offset can be absorbed into `desired_rho` later.
- A pure axial shift of the part is seen through the corner position; a lateral shift or a
  yaw through the edge line. A nudge faster than the arm follows can drop the edge: it is
  re-seeded from the last tracked line (`edge_snap_radius` 120 px in the demo). If it stays
  lost for `edge_lost_timeout_sec` the node goes to `FAILED`.
- Tunables are in `config/ur30_ibvs.yaml` (stage 4) and `config/ur30_ibvs_demo.yaml`.

## Tests

```bash
colcon test --packages-select ur30_ibvs   # gtest + pytest, no simulator needed
ros2 launch ur30_ibvs ur30_ibvs.launch.py test_cases:=3   # random-pose hold test in Gazebo
```
