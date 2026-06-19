# Vendor SDK setup

The public repository intentionally excludes vendor SDK source files and
official demo programs.

For local real-robot execution:

1. Obtain the Python SDK directly from the robot vendor or your licensed local
   SDK package.
2. Copy the required SDK Python package into `Robotic_Arm/`.
3. Verify these imports work:

```bash
python -c "from Robotic_Arm.rm_robot_interface import *; from Robotic_Arm.rm_ctypes_wrap import rm_realtime_push_config_t, rm_thread_mode_e; print('SDK import OK')"
```

Keep SDK files local. The `.gitignore` rules are set up so these files should
not be committed.
