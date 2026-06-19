# Vendor SDK placeholder

This directory is intentionally empty in the public repository.

To run the real robot scripts, obtain the Python SDK from the robot vendor
under the vendor's own license, then copy the required SDK package files here
locally. Do not commit those files to this repository.

The control scripts expect imports shaped like:

```python
from Robotic_Arm.rm_robot_interface import *
from Robotic_Arm.rm_ctypes_wrap import rm_realtime_push_config_t, rm_thread_mode_e
```
