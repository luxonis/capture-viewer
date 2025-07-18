import subprocess

# Common parameters
env = "dai3"
capture_script = "capture/dai3_stereo_capture_port.py"
capture_name = "test_4_device"

# Device configurations
devices = [
    {"ip": "192.168.50.102", "settings": "dai3", "port": 5555},
    {"ip": "192.168.50.103", "settings": "dai3", "port": 5556},
    {"ip": "192.168.50.118", "settings": "dai3_raw", "port": 5557},
    {"ip": "192.168.50.130", "settings": "dai3_raw", "port": 5558}
]

processes = []

for dev in devices:
    args = [
        "conda", "run", "-n", env, "python", capture_script,
        dev["settings"], capture_name,
        "--ip", dev["ip"],
        "--port", str(dev["port"])
    ]
    print(f"Launching device at {dev['ip']} on port {dev['port']}")
    p = subprocess.Popen(args)
    processes.append(p)

# Optional: wait for all to complete (or remove this if you just want to launch)
for p in processes:
    p.wait()
