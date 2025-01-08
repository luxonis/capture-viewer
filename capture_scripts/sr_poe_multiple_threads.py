import threading
import subprocess
import os

# List of device IDs to pass to the command
mxids = ['14442C1091F5D9E700', '14442C10417EB3CF00', '14442C10F10AC8D600']


def worker(mxid, i):
    # Define the command to run
    command = [
        "python", "sr_poe_capture.py",
        "/home/katka/PycharmProjects/capture-viewer/settings_jsons/sr_poe_settings_default.json",
        "view", "--device-ip", mxid, "--autostart", "5", "--frame-sync-num", str(i)
    ]

    env = os.environ.copy()  # Start with the current environment variables
    # env["XLINK_LEVEL"] = "debug"
    # env["DEPTHAI_CONNECT_TIMEOUT"] = "10000"  # https://docs.luxonis.com/software/depthai-components/device/#Device-Environment%20Variables

    # Run the command with the modified environment
    subprocess.run(command, env=env)

    # Run the command using subprocess
    subprocess.run(command)


if __name__ == '__main__':
    threads = []
    for i, mxid in enumerate(mxids):
        thread = threading.Thread(target=worker, args=(mxid, i))
        thread.start()
        threads.append(thread)

    # Wait for all threads to finish
    for t in threads:
        t.join()
