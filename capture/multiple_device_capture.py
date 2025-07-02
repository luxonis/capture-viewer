import subprocess
import time

now = time.time()
start_time = time.strftime('%H:%M:%S', time.localtime(now + 20))
end_time = time.strftime('%H:%M:%S', time.localtime(now + 20 + 30))
print(start_time)
print(end_time)

capture_name = "alternating"
settings = 'dai3_raw'


# Define the scripts, their environments, and arguments
jobs = [

    {
        "env": "dai3",
        "script": "capture/da3_alternating.py",
        "args": [settings, capture_name, "--alternating_capture", "True", "--ip", "184430106176351300"]
    },
    {
        "env": "dai3",
        "script": "capture/da3_alternating.py",
        "args": [settings, capture_name, "--alternating_capture", "True", "--ip", "1844301051F3860E00"]
    },
    # {
    #     "env": "dai3",
    #     "script": "capture/da3_alternating.py",
    #     "args": [settings, capture_name, "--alternating_capture", "True", "--ip", "184430106176351300"]
    # },
    # {
    #     "env": "dai3",
    #     "script": "capture/da3_alternating.py",
    #     "args": [settings, capture_name, "--alternating_capture", "True", "--ip", "1844301051F3860E00"]
    # }
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