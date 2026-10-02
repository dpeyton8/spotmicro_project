#!/usr/bin/env bash
set -euo pipefail

python3 - <<'PY'
import os
import signal

packages = {"spot_micro_mujoco_sim", "spot_micro_motion_cmd"}
launch_pids = []

for entry in os.listdir("/proc"):
    if not entry.isdigit():
        continue
    pid = int(entry)
    if pid == os.getpid():
        continue
    try:
        raw_args = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
        args = [part.decode(errors="replace") for part in raw_args if part]
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    if "launch" in args and packages.intersection(args):
        launch_pids.append(pid)

if not launch_pids:
    print("No SpotMicro simulation or motion launch is running.")
else:
    for pid in launch_pids:
        try:
            os.kill(pid, signal.SIGINT)
            print(f"Sent Ctrl+C to SpotMicro launch PID {pid}")
        except ProcessLookupError:
            pass
PY