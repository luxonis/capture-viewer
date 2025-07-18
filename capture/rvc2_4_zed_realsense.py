import subprocess
import time

now = time.time()
start_time = time.strftime('%H:%M:%S', time.localtime(now + 30))
end_time = time.strftime('%H:%M:%S', time.localtime(now + 60))
print(start_time)
print(end_time)

capture_name = "attempt_1"


# Define the scripts, their environments, and arguments
jobs = [

    {
        "env": "zed",
        "script": "capture/zed_capture.py",
        "args": [capture_name, "--output", "DATA", "--settings", "settings_jsons/zed_settings.json"]
    },
    {
        "env": "realsense",
        "script": "capture/realsense_capture.py",
        "args": [capture_name, "--output", "DATA", "--settings", "settings_jsons/rs_settings.json"]
    },
    {
        "env": "dai3",
        "script": "capture/rvc4_capture.py",
        "args": ["rvc4", capture_name, "--ip", "192.168.50.101"]
    },
    {
        "env": "capture_viewer",
        "script": "capture/dai2_stereo_capture.py",
        "args": ["default", capture_name, "--devices", "192.168.50.102"]
    }
]

processes = []

for job in jobs:
    job['args'].append("--autostart_time")
    job['args'].append(str(start_time))
    job['args'].append("--autostart_end")
    job['args'].append(str(end_time))
    args_str = " ".join(job["args"])
    cmd = f"conda run -n {job['env']} python {job['script']} {args_str}"
    p = subprocess.Popen(cmd, shell=True)
    processes.append(p)

for p in processes:
    p.wait()
