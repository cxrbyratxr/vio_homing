# Local checks 2026-10-17

29 existing Python3 unit tests PASS.
All Python sources parse; launch/package XML parses; wireless include args exist.
Homing transport/alignment remain disabled; single-buffer camera regression absent.
Both shell scripts: bash -n PASS, run_wireless.sh --help PASS.
Actual bridge connect method executed with fake MAVLink/ROS:disabled/100/200Hz
and reconnect issue correct command511,message105,interval10000/5000us.
No hardware commands sent by these local checks.

ROS Melodic/Python2.7 runtime, camera/QGC over AP and real ACK remain to be
verified on Jetson after transfer. No package installation attempted locally.
