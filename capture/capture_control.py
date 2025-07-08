import sys
import os
import json
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import tkinter as tk
from tkinter import ttk
import zmq
import threading
import time
import subprocess

# ----- ZeroMQ setup -----
context = zmq.Context()

# Device configurations from JSON
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "devices_config.json")
with open(CONFIG_FILE) as f:
    devices_config = json.load(f)

env = "dai3"
capture_script = os.path.join(os.path.dirname(__file__), "dai3_port_capture.py")
capture_name = "test_4_device"

# ----- Command sender -----
def send_command(port, cmd):
    try:
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 sec timeout
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect(f"tcp://localhost:{port}")
        socket.send_json({"cmd": cmd})
        response = socket.recv_json()
        return response
    except zmq.error.Again:
        return {"status": "timeout"}
    except Exception as e:
        return {"status": "error", "detail": str(e)}
    finally:
        socket.close()

# ----- GUI Application -----
class MultiDeviceControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-Device Capture Controller")
        self.device_ports = {f"Device {i+1}": int(port) for i, port in enumerate(devices_config.keys())}
        self.status_vars = {name: tk.StringVar(value="Disconnected") for name in self.device_ports}
        self.status_labels = {}
        self.restart_buttons = {}
        self.running = False
        self.all_ready = False

        self.build_ui()
        self.poll_statuses()

    def build_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky="nsew")

        ttk.Button(main_frame, text="Launch Capture", command=self.launch_all_devices).grid(row=0, column=0, columnspan=3, pady=(0, 10))

        row = 1
        for device, port in self.device_ports.items():
            ttk.Label(main_frame, text=f"{device} (Port {port})").grid(row=row, column=0, sticky="w")
            label = ttk.Label(main_frame, textvariable=self.status_vars[device])
            label.grid(row=row, column=1, sticky="w")
            self.status_labels[device] = label

            restart_btn = ttk.Button(main_frame, text="Restart", command=lambda p=port: self.restart_device(p))
            restart_btn.grid(row=row, column=2, padx=5)
            restart_btn.grid_remove()  # hidden by default
            self.restart_buttons[device] = restart_btn

            row += 1

        # Start and Exit buttons
        ttk.Button(main_frame, text="Start Capture Sequence", command=self.start_sequence).grid(row=row, column=0, pady=10)
        ttk.Button(main_frame, text="End Capture (Exit)", command=self.end_capture).grid(row=row, column=1, pady=10)

    def poll_statuses(self):
        for device, port in self.device_ports.items():
            threading.Thread(target=self.update_status, args=(device, port), daemon=True).start()
        self.check_launch_ready()
        self.root.after(5000, self.poll_statuses)  # Repeat every 5s

    def check_launch_ready(self):
        self.all_ready = all(self.status_vars[device].get().lower() == "ready" for device in self.device_ports)

    def update_status(self, device, port):
        response = send_command(port, "status")
        status = response.get("status", "unknown")
        self.status_vars[device].set(status)

        label = self.status_labels[device]
        if status.lower() == "ready":
            label.config(foreground="green")
            self.restart_buttons[device].grid_remove()
        elif status.lower() in ["disconnected", "error", "timeout", "interrupted"]:
            label.config(foreground="red")
            self.restart_buttons[device].grid()
        else:
            label.config(foreground="black")
            self.restart_buttons[device].grid_remove()

    def start_sequence(self):
        if not self.all_ready:
            return
        self.running = True
        threading.Thread(target=self._run_loop_sequence, daemon=True).start()

    def _run_loop_sequence(self):
        while self.running:
            time.sleep(0.5)
            for device, port in self.device_ports.items():
                self.status_vars[device].set("Capturing (off)")
                send_command(port, "capture_frame")

            for device, port in self.device_ports.items():
                self.status_vars[device].set("Projector ON")
                send_command(port, "projector_on")
                time.sleep(1)
                self.status_vars[device].set("Capturing (on)")
                send_command(port, "capture_frame")
                self.status_vars[device].set("Projector OFF")
                send_command(port, "projector_off")
                self.status_vars[device].set("Done")
        threading.Thread(target=self._finalize_exit, daemon=True).start()

    def end_capture(self):
        self.running = False

    def _finalize_exit(self):
        time.sleep(0.5)  # Ensure any current capture finishes
        for device, port in self.device_ports.items():
            response = send_command(port, "exit")
            if response.get("status") == "shutting_down":
                self.status_vars[device].set("Shutting down")
                self.status_labels[device].config(foreground="red")

    def restart_device(self, port):
        config = devices_config.get(str(port))
        if not config:
            print(f"No config found for port {port}")
            return

        args = [
            "conda", "run", "-n", env, "python", capture_script,
            config["settings"], capture_name,
            "--ip", config["ip"],
            "--port", str(port)
        ]
        print(f"Restarting device on port {port} with IP {config['ip']}")
        subprocess.Popen(args)

    def launch_all_devices(self):
        for port, config in devices_config.items():
            args = [
                "conda", "run", "-n", env, "python", capture_script,
                config["settings"], capture_name,
                "--ip", config["ip"],
                "--port", str(port)
            ]
            print(f"Launching device on port {port} with IP {config['ip']}")
            subprocess.Popen(args)

# ----- Run GUI -----
if __name__ == "__main__":
    root = tk.Tk()
    app = MultiDeviceControlApp(root)
    root.mainloop()
