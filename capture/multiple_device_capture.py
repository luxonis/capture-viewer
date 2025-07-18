import subprocess
import time

now = time.time()
start_time = time.strftime('%H:%M:%S', time.localtime(now + 30))
end_time = time.strftime('%H:%M:%S', time.localtime(now + 30 + 30))
print(start_time)
print(end_time)

capture_name = "test_4_device"
settings_RVC4 = 'dai3_raw'
settings_RVC2 = 'dai3'


# Define the scripts, their environments, and arguments
jobs = [

    {
        "env": "dai3",
        "script": "capture/dai3_stereo_capture_alternating.py",
        "args": [settings_RVC2, capture_name, "--alternating_capture", "True", "--ip", "192.168.50.102"]
    },
    {
        "env": "dai3",
        "script": "capture/dai3_stereo_capture_alternating.py",
        "args": [settings_RVC2, capture_name, "--alternating_capture", "True", "--ip", "192.168.50.103"]
    },
    {
        "env": "dai3",
        "script": "capture/dai3_stereo_capture_alternating.py",
        "args": [settings_RVC4, capture_name, "--alternating_capture", "True", "--ip", "192.168.50.118"]
    },
    {
        "env": "dai3",
        "script": "capture/dai3_stereo_capture.py",
        "args": [settings_RVC4, capture_name, "--ip", "192.168.50.130"]
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