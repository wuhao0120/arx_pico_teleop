def main():
    print("""
==================================================
 ARX VR Teleoperation - Command Reference
==================================================

Main Entry:
  ./start_recording.sh [debug|record]
                  Full automatic data recording (starts controllers + RPC + data collection in separate terminals)
  python scripts/core/run_record_arx.py
                  Direct run data recording (controllers and RPC must be already running)

Visualization:
  python scripts/core/run_visualize.py
                  Visualize recorded dataset with Rerun

Tool Commands:
  python scripts/tools/check_dataset_info.py
                  Check local dataset information
  python scripts/tools/rs_devices.py
                  List connected RealSense cameras with serial numbers

Shell Tools:
  ./scripts/tools/map_gripper.sh
                  Map Gripper Serial Port to /dev/ttyUSB

Test Commands:
  ./scripts/tests/run_arx_test.sh
                  Run ARX module tests (no hardware connection required)
  python scripts/tests/test_arx_modules.py
                  Import test for all ARX modules
  python scripts/tests/test_xr_connection.py
                  Test connection to VR headset (XRoboToolkit)
  python scripts/tests/gripper_ctrl.py
                  Test gripper control (open/close)
  python scripts/tests/test_lift_move_up.py
                  Test LIFT chassis move up/down

Environment Setup:
  source scripts/env/setup_arx_r5.sh
                  Setup environment for ARX LIFT + dual R5
  source scripts/env/setup_arx_x5.sh
                  Setup environment for ARX LIFT + dual X5

--------------------------------------------------
 Tip: Use 'python -m scripts.help.help_info' to see this summary.
==================================================
""")
