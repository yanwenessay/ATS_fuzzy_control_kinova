# RealMan 65B ATS Fuzzy Control Code

Author: WenYan

Repository name: `realman_65b_ats_fuzzy_control_code_opensource`

This repository contains a cleaned GitHub release of a 6-DOF ATS adaptive fuzzy joint-tracking controller for a RealMan 65B robot arm. The main program is:

```text
custom_robot_ats_control_6dof_dynamic.py
```

The release keeps only the ATS fuzzy-control runtime code and the minimum local configuration/documentation needed to run it. Other experiment, comparison, training, simulation, generated-output, cache, backup, vendor PDF, vendor SDK source, and official demo files have been removed.

## File Layout

```text
.
|-- custom_robot_ats_control_6dof_dynamic.py  # main 6-DOF ATS control entry
|-- control_main.py                           # ATS adaptive fuzzy control law
|-- ts_fuzzy_output.py                        # TS fuzzy output wrapper
|-- fuzzy_membership_fcn.py                   # fuzzy membership calculation
|-- fuzzyoutput.py                            # fuzzy rule output
|-- DiscreteIntegrator.py                     # adaptive parameter integrator
|-- robot_config.py                           # private robot IP/port loader
|-- config.example.json                       # public config template
|-- Robotic_Arm/                              # vendor SDK placeholder only
`-- docs/                                     # safety, SDK, privacy notes
```

## Environment

Recommended Python version: 3.10 or 3.11.

```bash
conda create -n ats_fuzzy python=3.10
conda activate ats_fuzzy
pip install -r requirements.txt
```

Runtime Python dependencies:

- `numpy`
- `matplotlib`

The real-robot script also needs the robot vendor Python SDK, but that SDK is intentionally not included in this repository.

## Vendor SDK

This release does not publish vendor SDK code or official demo code. To run the robot script locally, obtain the SDK through the vendor-authorized channel and place the required Python package files in `Robotic_Arm/`.

The main script expects these imports to work:

```python
from Robotic_Arm.rm_robot_interface import *
from Robotic_Arm.rm_ctypes_wrap import rm_realtime_push_config_t, rm_thread_mode_e
```

`Robotic_Arm/` is ignored by Git except for its placeholder README and `.gitignore`, so SDK files should remain local.

## Robot Connection Config

Do not write a real robot IP into source code or the README. Copy the public template and edit only your local file:

```bash
cp config.example.json config.local.json
```

Windows PowerShell:

```powershell
Copy-Item config.example.json config.local.json
```

Then edit `config.local.json` locally:

```json
{
  "robot_ip": "YOUR_ROBOT_IP",
  "robot_port": 8080
}
```

You can also use environment variables:

```bash
export CUSTOM_ROBOT_IP="YOUR_ROBOT_IP"
export CUSTOM_ROBOT_PORT="8080"
```

Windows PowerShell:

```powershell
$env:CUSTOM_ROBOT_IP="YOUR_ROBOT_IP"
$env:CUSTOM_ROBOT_PORT="8080"
```

`config.local.json` is ignored by `.gitignore`.

## Run

After installing dependencies, adding the vendor SDK locally, and setting the private robot config:

```bash
python custom_robot_ats_control_6dof_dynamic.py
```

The script connects to the robot, configures UDP state feedback, moves to the configured start joint pose, and runs the ATS adaptive fuzzy controller. Review the source before running on hardware, especially the start pose and current-loop behavior.

## Safety

This project can command a real robot arm in current-loop/joint-control workflows. Before every hardware run:

- Confirm the configured IP points to the intended robot.
- Keep the emergency stop reachable.
- Keep people and loose objects outside the workspace.
- Confirm the start pose is safe for your hardware and tooling.
- Start with conservative speed/current/motion ranges.
- Stop immediately if tracking error, velocity, current, temperature, or motion looks abnormal.

See `docs/SAFETY.md` for the short checklist.

## Release Notes

This directory is prepared for GitHub publication with privacy and copyright concerns in mind:

- No real robot IP is stored in tracked files.
- No vendor SDK source, official demo program, or vendor PDF is included.
- Generated outputs and local config are ignored.
- A license has not been selected yet; see `LICENSE_NOTICE.md` before public release.
