#!/bin/bash
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
python - <<'PY'
import rosgraph, sys
if rosgraph.is_master_online():
    sys.exit('ROS master already running.')
PY
exec roscore
