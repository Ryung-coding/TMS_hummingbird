# TMS HummingBird — Thrust & Wall Effect Experiment

ROS 2 Jazzy workspace for fixed HummingBird thrust and wall effect experiments. The GUI sends normalized motor commands through PX4 Offboard direct actuator control, sends servo angles to the Dynamixel SDK node, and records the ROBOTOUS `/RFT_FORCE1` wrench. The wall distance is changed manually between full experiments.

## Packages and connections

- `src/px4_msgs`: the four PX4 messages used by this workspace: `ActuatorMotors`, `OffboardControlMode`, `VehicleCommand`, and `VehicleStatus`. Their definitions match the local `HB_v1.17` source.
- `src/tms_hummingbird/include/params.hpp`: PX4 command IDs, timeouts, Dynamixel IDs, directions, limits, gains and register settings.
- `src/tms_hummingbird/include/utils.hpp`: existing LPF used for startup zero return and smooth runtime servo commands.
- `src/tms_hummingbird/src/bldc_control.cpp`: Offboard heartbeat, explicit Arm/Disarm, four equal motor commands, GUI `u max` enforcement and timeout handling.
- `src/tms_hummingbird/src/dynamixel_control.cpp`: Dynamixel initialization, zero return, goal position writes, connection state and shutdown.
- `src/tms_hummingbird/scripts/tms_gui.py`: PySide6 GUI, automatic experiment sequence, zero/Yaw correction, graphs and CSV writing.
- `src/rft_sensor_serial`: copied from the working ROBOTOUS UART driver workspace. Its parameters, serial protocol, and `/RFT_FORCE1` publisher are retained.
- `src/tms_hummingbird/launch/tms_experiment.launch.py`: starts the F/T driver, GUI, and both control nodes in one launch. The F/T USB port is passed to the driver as `DEV_NAMES`.

```text
GUI ── /tms/u_max, /tms/motor_target ──> bldc_control ── /fmu/in/actuator_motors ──> PX4
    ├─ /tms/offboard_request, /tms/arm_request, /tms/emergency_stop ──> bldc_control
    └─ /tms/servo_target_deg ──> dynamixel_control ──> Dynamixel IDs 0–5

PX4 /fmu/out/vehicle_status ──> bldc_control + GUI
bldc_control /tms/bldc_state, /tms/motor_output ──> GUI
Dynamixel node /tms/servo_ready ──> bldc_control + GUI
Dynamixel node /tms/servo_commanded_deg ──> GUI (filtered command, not measured angle)
rft_sensor_serial /RFT_FORCE1 ──> bldc_control + GUI
```

`/tms/servo_ready` reports that the node is receiving and successfully writing angle commands. Runtime servo targets pass through the existing `utils::LPF` with a 2 s 99% response time and a 60°/s command rate limit. `/tms/servo_commanded_deg` reports the filtered target actually sent to the SDK at 20 Hz. Servo position is read once during the existing safe startup zero sequence; there is no periodic position read or measured-angle topic. The GUI waits until all six filtered commands are within 0.5° of target for 0.5 s before motor Ramp. This is command settling, not measured physical position.

## Build and shell aliases

The following aliases are in `~/.bashrc`:

```bash
cs_tms   # cd, clear older ROS overlays, source Jazzy and TMS
cm_tms   # same clean environment, build with colcon, then source TMS
```

Open a new terminal or run `source ~/.bashrc` to load the aliases, then use `cm_tms` for a rebuild. Both aliases remove the older `px4_hummingbird` overlay from that shell before selecting TMS, so its reduced `px4_msgs` package is used consistently. The equivalent direct build is:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/tms_hummingbird
colcon build --symlink-install
source install/local_setup.bash
```

The GUI uses system Python packages `PySide6`, `pyqtgraph`, `numpy` and ROS Jazzy `rclpy`.

## Run and connection order

Use separate USB devices for the F/T sensor and Dynamixel. The only launch arguments are `ft_port` (default `/dev/ttyUSB0`) and `dxl_port` (default `/dev/ttyUSB1`); identical resolved paths are rejected. Stable `/dev/serial/by-id/...` names are preferred. When the F/T port exists, launch runs `setserial <ft_port> low_latency` as in the working ROBOTOUS launch. Each node starts independently. A missing PX4 DDS connection or USB device shows as disconnected/not ready in the GUI; it does not prevent the GUI and other available nodes from running.

1. Start PX4 with the intended `HB_v1.17` firmware and start the MicroXRCEAgent for its transport. For local UDP 8888: `MicroXRCEAgent udp4 -p 8888`. PX4, agent, and ROS must use the same DDS domain. Do not run another Offboard actuator publisher at the same time.
2. Open a terminal and run:

```bash
cs_tms
ros2 launch tms_hummingbird tms_experiment.launch.py \
  ft_port:=/dev/ttyUSB0 dxl_port:=/dev/ttyUSB1
```

The launch starts `rft_sensor_serial`, `bldc_control`, `dynamixel_control`, and `tms_gui.py`. It does not start PX4 or MicroXRCEAgent. The F/T driver is inside this workspace; no separate source or launch is needed. Without `cs_tms`, source `/opt/ros/jazzy/setup.bash` and `~/Desktop/tms_hummingbird/install/local_setup.bash` first.

For a **sensor-only live plot**, use the same command with only the F/T sensor connected. Dynamixel initialization may report its missing USB port and exit; `bldc_control` stays not ready. The GUI continues plotting `/RFT_FORCE1`. With only Dynamixel attached, the same command lets you use **Manual CHK → Dynamixel CHK** while the other connections show unavailable. There are no `enable_*` launch arguments.

Measured Force/Torque plots update whenever `/RFT_FORCE1` arrives, including when PX4 and Dynamixel are disconnected. The two disturbance plots show `Baseline not loaded` until a matching baseline and active condition exist. Motor/servo check and START still require their own readiness.

Connection path:

```text
ROBOTOUS USB -> rft_sensor_serial -> /RFT_FORCE1 -> GUI plots + bldc_control
Dynamixel USB <- dynamixel_control <- /tms/servo_target_deg <- GUI
PX4 <-> MicroXRCEAgent <-> /fmu/out/vehicle_status -> GUI + bldc_control
GUI -> /tms/u_max, /tms/motor_target, /tms/offboard_request,
       /tms/arm_request, /tms/emergency_stop -> bldc_control
bldc_control -> /fmu/in/offboard_control_mode, /fmu/in/vehicle_command,
                /fmu/in/actuator_motors -> PX4
bldc_control -> /tms/bldc_state, /tms/motor_output -> GUI
Dynamixel -> /tms/servo_ready -> GUI + bldc_control
Dynamixel -> /tms/servo_commanded_deg -> GUI servo settling gate
```

Check the ROS graph with `ros2 node list`, `ros2 topic list`, `ros2 topic hz /RFT_FORCE1`, and `ros2 topic info /fmu/in/actuator_motors` in a TMS-sourced shell. Do not publish actuator test messages from another node while `bldc_control` owns the PX4 command topics.

## GUI operation

The **Experiment / Live F/T** tab contains the connection indicators, calibration, full experiment settings, and four live plots. Raw and corrected/Yaw values update with F/T samples even if PX4, BLDC, or Dynamixel is offline. Wall disturbance plots require a valid baseline and an active experiment condition.

The **Manual CHK** tab separates deliberate actuator checks from the automatic experiment. Dynamixel CHK allows Alpha/Beta commands within the mechanical angle limits. Turn on **Dynamixel CHK enable**, set angles, and click **Apply Alpha / Beta**; **Servo 0°** or turning the check off returns the target to zero. This check only needs `dynamixel_control` ready. The same LPF and rate limit apply to Manual CHK. The tab shows filtered command feedback, which is not a measured angle.

For PX4, turn on **Offboard CHK enable** and, after the connection and calibration checks pass, click **Request Offboard**. Confirm the state label, then click **Arm** explicitly. Set a normalized motor `u` no higher than **u max**, then click **Apply motor u**. The GUI keeps sending that target at 20 Hz until **Motor 0**, **Disarm / exit Offboard**, disabling Offboard CHK, or **Emergency Stop**. Connection loss or a reduced u max cancels the held command and requests Disarm. Manual CHK must be off before START can be used. This tab does not arm, enter Offboard, or apply a nonzero command automatically.

1. With motors disarmed, run **Zero Calibration**. It averages all six raw channels over the selected interval (5 s default). Set Sensor Yaw numerically; `Reset Zero` clears the offset.
2. Set **u max** in `[0,1]` as the independent normalized output ceiling. Set Motor Min/Max/Step with Motor Max no greater than u max. The default u max is 0.35; it is editable. The GUI sends it to `bldc_control` at 20 Hz, and the node rejects commands over it. The GUI displays command percentage and model-based estimated thrust.
3. Set Alpha/Beta Enable and their degree ranges. Disabled axes stay at 0°. Set Ramp, Settling and Measurement time.
4. For `d=inf`, click **Enter Offboard**, confirm the PX4 mode, then click **Arm** explicitly. **START never Arms.** START executes all Alpha × Beta × motor combinations and saves the baseline.
5. Move the wall manually. Enable Wall, enter distance `d` in meters, select the completed `summary_d_inf.csv`, and repeat with identical conditions. An absent or invalid baseline disables START.

For each angle combination, motor command returns to zero and stays there for at least 0.3 s before moving the servos. The six servo targets are `[-beta, +beta, +alpha, +alpha, -alpha, -alpha]` for IDs 0–5. The GUI waits for all six filtered commands to be within 0.5° of target continuously for 0.5 s, with a 12 s command-settling timeout. It then runs motor Ramp → Settling → Measurement. This gate checks commanded motion only, because no periodic measured servo angle is read. Only Measurement samples enter statistics. STOP and Emergency Stop preserve files already written, command motor zero, request Disarm and Offboard exit, and send servo zero. The GUI and ROS callbacks use separate threads.

## Coordinates, motor model and results

Zero correction is applied in sensor coordinates. Force and torque each use the same Yaw rotation:

`x' = cos(yaw)*x - sin(yaw)*y`, `y' = sin(yaw)*x + cos(yaw)*y`, `z' = z`.

Positive Yaw rotates sensor +X toward output +Y. Raw, zero-corrected and yaw-rotated six-axis data are retained. The plotted live wall disturbance is the current rotated sample minus the same condition's `d=inf` mean. Final disturbance is the difference of Measurement means. The GUI requires matching Alpha/Beta/Motor conditions, time settings, u max, Yaw, Zero Calibration state, and servo smoothing settings in the baseline metadata.

Local PX4 `ControlAllocator.cpp` uses `u = sqrt(F_rotor/F_max)` with `F_max = 1961.04 gf * 9.80665e-3 N/gf`. The GUI therefore displays `F_rotor ≈ 19.2312329276*u² N` and `F_total ≈ 4*F_rotor`. These are model estimates, not measured thrust.

Each run gets a unique timestamp directory under `results/d_inf/` or `results/d_<distance>m/`. Raw rows are flushed as received; completed Summary and Disturbance rows are flushed and synced. `metadata.json` records settings, zero offsets, baseline, condition progress and final state. Interrupted runs retain already written data.

- Raw CSV: timestamp, sensor timestamp, phase, condition, Alpha/Beta/motor commands, motor node output, six servo **target** angles, six filtered **commanded** angles, and raw/corrected/rotated six-axis values. Commanded angles are not measured angles.
- Summary CSV: condition, estimated thrust, sample count, validity, and six-axis mean/sample variance/standard deviation.
- Disturbance CSV in Wall mode: condition and six differences of Wall and baseline means.

The local `HB_v1.17` source already subscribes to `/fmu/in/actuator_motors` and checks armed Offboard direct actuator mode and four finite normalized values. No PX4 firmware file was changed. Before hardware use, confirm the flashed firmware, rotor mapping and motor output behavior, servo mechanical limits and direction, USB device assignments, F/T units, Yaw sign and the PX4 Offboard loss action. No motor, servo or sensor hardware was run while implementing this workspace.
