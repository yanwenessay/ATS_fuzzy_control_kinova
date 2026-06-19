# Safety notes

This project can command a real robot in current-loop or joint-control modes.
Treat every real-robot run as a physical experiment.

- Validate controller behavior in simulation first.
- Keep the emergency stop reachable.
- Keep people and loose objects outside the robot workspace.
- Verify the configured IP and port point to the intended robot only.
- Start with conservative speeds, currents, and motion ranges.
- Confirm the `[0, 0, 0, 0, 0, 0]` start pose is safe for your hardware, tooling, and workspace.
- Stop immediately if tracking error, joint velocity, current, or temperature looks abnormal.
