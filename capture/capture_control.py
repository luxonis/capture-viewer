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

import os
env = os.environ.get("CONDA_DEFAULT_ENV") or (
    os.path.basename(os.environ.get("VIRTUAL_ENV")) if os.environ.get("VIRTUAL_ENV") else "base"
)
print("Environment:", env)

capture_script = os.path.join(os.path.dirname(__file__), "dai3_port_capture.py")

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
        self.max_captures_var = tk.StringVar(value="Unlimited")

        self.build_ui()
        self.poll_statuses()

    def build_ui(self):
        self.message_var = tk.StringVar(value="Idle")
        self.capture_name_var = tk.StringVar(value="test_capture")
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky="nsew")

        ttk.Label(main_frame, text="Capture Name:").grid(row=0, column=0, sticky="e")
        entry = ttk.Entry(main_frame, textvariable=self.capture_name_var)
        entry.grid(row=0, column=1, sticky="w")

        self.launch_button = ttk.Button(main_frame, text="Launch Capture", command=self.launch_all_devices, state="enabled")
        self.launch_button.grid(row=0, column=2, pady=(0, 10), padx=5)

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

        self.start_sequence_button = ttk.Button(main_frame, text="Start Capture Sequence", command=self.start_sequence, state="disabled")
        self.start_sequence_button.grid(row=row, column=0, pady=10)
        ttk.Button(main_frame, text="End Capture (Exit)", command=self.end_capture).grid(row=row, column=1, pady=10)

        # Message Label
        row += 1

        ttk.Label(main_frame, text="Max Captures:").grid(row=row, column=0, sticky="e")
        max_caps_entry = ttk.Entry(main_frame, textvariable=self.max_captures_var, foreground='gray')
        max_caps_entry.grid(row=row, column=1, sticky="w")

        def on_focus_in(event):
            if self.max_captures_var.get() == "Unlimited":
                self.max_captures_var.set("")
                max_caps_entry.config(foreground='black')

        def on_focus_out(event):
            if self.max_captures_var.get() == "":
                self.max_captures_var.set("Unlimited")
                max_caps_entry.config(foreground='gray')

        max_caps_entry.bind("<FocusIn>", on_focus_in)
        max_caps_entry.bind("<FocusOut>", on_focus_out)

        row += 1
        ttk.Label(main_frame, textvariable=self.message_var, foreground="blue").grid(row=row, column=0, columnspan=3,
                                                                                     sticky="w", pady=(10, 0))

    def poll_statuses(self):
        for device, port in self.device_ports.items():
            threading.Thread(target=self.update_status, args=(device, port), daemon=True).start()
        if not self.running: self.check_launch_ready()
        self.root.after(500, self.poll_statuses)

    def check_launch_ready(self):
        self.all_ready = all(self.status_vars[device].get().lower() == "ready" for device in self.device_ports)
        if self.all_ready:
            self.start_sequence_button.config(state="enabled")
            self.set_message("Devices Ready!")

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

    def set_message(self, msg):
        self.message_var.set(msg)

    def start_sequence(self):
        if not self.all_ready:
            return
        self.set_message("Capturing...")
        self.running = True
        threading.Thread(target=self._run_loop_sequence, daemon=True).start()

    def _run_loop_sequence(self):
        try:
            max_captures = self.max_captures_var.get()
            max_captures = int(max_captures) if max_captures.strip().isdigit() else None
        except Exception:
            max_captures = None

        count = 0
        while self.running and (max_captures is None or count < max_captures):
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

            count += 1

        self.running = False
        threading.Thread(target=self._finalize_exit, daemon=True).start()

    def end_capture(self):
        if not self.running:
            threading.Thread(target=self._finalize_exit, daemon=True).start()
        self.running = False
        self.set_message("Capture ending..")

    def _finalize_exit(self):
        time.sleep(0.5)
        for device, port in self.device_ports.items():
            response = send_command(port, "exit")
            if response.get("status") == "shutting_down":
                self.status_vars[device].set("Shutting down")
                self.status_labels[device].config(foreground="red")

            self.set_message("Capture ended")

    def restart_device(self, port):
        capture_name = self.capture_name_var.get()
        if " " in capture_name:
            print("Capture name must not contain spaces.")
            self.set_message("Capture name must not contain spaces.")
            return
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
        self.set_message("Launching devices...")
        capture_name = self.capture_name_var.get()
        if " " in capture_name:
            print("Capture name must not contain spaces.")
            self.set_message("Capture name must not contain spaces.")
            return
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
