import sys
import os
import json
import re
import shutil
from datetime import datetime
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import zmq
import threading
import time
import subprocess

# ----- ZeroMQ setup -----
context = zmq.Context()

# Device configurations from JSON
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "capture_control_config.json")
with open(CONFIG_FILE) as f:
    devices_config = json.load(f)

import os
python_path = os.environ.get("PYTHON_BIN", None)
if python_path is None:
    env = os.environ.get("CONDA_DEFAULT_ENV") or (
        os.path.basename(os.environ.get("VIRTUAL_ENV")) if os.environ.get("VIRTUAL_ENV") else "base"
    )
    python_path = ["conda", "run", "-n", env, "python"]
else:
    python_path = [python_path,]
print("Python path:", python_path)

capture_script = os.path.join(os.path.dirname(__file__), "dai3_stereo_capture_port_continuos.py")


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

def kill_process_on_port(port):
    try:
        output = subprocess.check_output(["lsof", "-t", f"-i:{port}"]).decode().strip()
        if output:
            pids = output.split("\n")
            for pid in pids:
                subprocess.run(["kill", "-9", pid])
                print(f"[Cleanup] Killed process {pid} on port {port}")
    except subprocess.CalledProcessError:
        # lsof returns non-zero if nothing found, that's fine
        print(f"[Cleanup] No process found on port {port}")


# ----- GUI Application -----
class MultiDeviceControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-Device Capture Controller")
        self.device_ports = {f"Device {i+1}": int(port) for i, port in enumerate(devices_config.keys())}
        self.status_vars = {name: tk.StringVar(value="Disconnected") for name in self.device_ports}
        self.status_history = {device: [] for device in self.device_ports}
        self.status_labels = {}
        self.device_labels = {}  # Store device name labels for updating
        self.restart_buttons = {}
        self.running = False
        self.save_one_inicialized = False
        self.max_captures_var = tk.StringVar(value="Unlimited")
        self.max_cycles_var = tk.StringVar(value="Unlimited")
        self.current_cycle = 0
        self.projector_sleep_var = tk.StringVar(value="5")
        self.capture_sleep_var = tk.StringVar(value="2")
        self.cycle_wait_var = tk.StringVar(value="3")
        
        # Configuration editor variables
        self.config_entries = {}
        self.original_config = devices_config.copy()
        self.config_modified = False

        self.build_ui()
        self.poll_statuses()

    def build_ui(self):
        style = ttk.Style()
        style.theme_use('default')  # Ensure theme allows custom styles
        style.configure("On.TButton", foreground="white", background="green")
        style.configure("Off.TButton", foreground="white", background="red")
        style.configure("Start.TButton", background="yellow", foreground="black")
        style.map("Start.TButton",
                  background=[("active", "#f7d200")],
                  foreground=[("disabled", "gray")])

        self.message_var = tk.StringVar(value="Idle")
        self.capture_name_var = tk.StringVar(value="test_capture")
        self.max_captures_var = tk.StringVar(value="Unlimited")

        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.grid(row=0, column=0, sticky="nsew")

        row = 0

        # === Row 0: Launch (left), Exit (right) ===
        self.launch_button = ttk.Button(main_frame, text="Launch Devices", command=self.launch_all_devices)
        self.launch_button.grid(row=row, column=0, sticky="w", padx=(0, 10))

        exit_button = ttk.Button(main_frame, text="Exit Devices", command=self.exit_devices)
        exit_button.grid(row=row, column=2, sticky="e")

        row += 1

        # === Row 1: Start Controls in their own frame ===
        start_controls_frame = ttk.LabelFrame(main_frame, text="Capture Controls", padding=(10, 5))
        start_controls_frame.grid(row=row, column=0, columnspan=3, sticky="ew", pady=(10, 5))

        # --- Capture Name ---
        ttk.Label(start_controls_frame, text="Capture Name:").grid(row=0, column=0, sticky="e")
        entry = ttk.Entry(start_controls_frame, textvariable=self.capture_name_var)
        entry.grid(row=0, column=1, sticky="w", padx=(0, 10))

        # --- Max Captures ---
        ttk.Label(start_controls_frame, text="Max Captures:").grid(row=0, column=2, sticky="e")
        max_caps_entry = ttk.Entry(start_controls_frame, textvariable=self.max_captures_var, foreground='gray',
                                   width=12)
        max_caps_entry.grid(row=0, column=3, sticky="w")

        # --- Max Cycles ---
        ttk.Label(start_controls_frame, text="Max Cycles:").grid(row=1, column=2, sticky="e")
        max_cycles_entry = ttk.Entry(start_controls_frame, textvariable=self.max_cycles_var, foreground='gray',
                                     width=12)
        max_cycles_entry.grid(row=1, column=3, sticky="w")

        # --- Timing Configuration ---
        timing_frame = ttk.LabelFrame(start_controls_frame, text="Timing (seconds)", padding=(5, 2))
        timing_frame.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(10, 5))

        ttk.Label(timing_frame, text="Projector Wait:").grid(row=0, column=0, sticky="e", padx=(0, 5))
        self.projector_sleep_entry = ttk.Entry(timing_frame, textvariable=self.projector_sleep_var, width=8)
        self.projector_sleep_entry.grid(row=0, column=1, sticky="w", padx=(0, 15))

        ttk.Label(timing_frame, text="Capture Duration:").grid(row=0, column=2, sticky="e", padx=(0, 5))
        self.capture_sleep_entry = ttk.Entry(timing_frame, textvariable=self.capture_sleep_var, width=8)
        self.capture_sleep_entry.grid(row=0, column=3, sticky="w", padx=(0, 15))

        ttk.Label(timing_frame, text="Cycle Wait:").grid(row=0, column=4, sticky="e", padx=(0, 5))
        self.cycle_wait_entry = ttk.Entry(timing_frame, textvariable=self.cycle_wait_var, width=8)
        self.cycle_wait_entry.grid(row=0, column=5, sticky="w")

        # Placeholder text behavior for Max Captures
        def on_focus_in_caps(event):
            if self.max_captures_var.get() == "Unlimited":
                self.max_captures_var.set("")
                max_caps_entry.config(foreground='black')

        def on_focus_out_caps(event):
            if self.max_captures_var.get() == "":
                self.max_captures_var.set("Unlimited")
                max_caps_entry.config(foreground='gray')

        max_caps_entry.bind("<FocusIn>", on_focus_in_caps)
        max_caps_entry.bind("<FocusOut>", on_focus_out_caps)

        # Placeholder text behavior for Max Cycles
        def on_focus_in_cycles(event):
            if self.max_cycles_var.get() == "Unlimited":
                self.max_cycles_var.set("")
                max_cycles_entry.config(foreground='black')

        def on_focus_out_cycles(event):
            if self.max_cycles_var.get() == "":
                self.max_cycles_var.set("Unlimited")
                max_cycles_entry.config(foreground='gray')

        max_cycles_entry.bind("<FocusIn>", on_focus_in_cycles)
        max_cycles_entry.bind("<FocusOut>", on_focus_out_cycles)

        # --- Start/Stop Controls ---
        self.start_sequence_button = ttk.Button(start_controls_frame, text="Start Alternating Capture",
                                                command=self.start_sequence, state="disabled", style="Start.TButton")
        self.start_sequence_button.grid(row=3, column=0, padx=5, pady=5)

        self.simple_sequence_button = ttk.Button(start_controls_frame, text="Start Simple Capture",
                                                 command=self.start_simple_sequence, state="disabled",
                                                 style="Start.TButton")
        self.simple_sequence_button.grid(row=3, column=1, padx=5, pady=5)

        self.projector_toggle_button = ttk.Button(start_controls_frame, text="Projector OFF",
                                                  command=self.toggle_projectors)
        self.projector_toggle_button.config(style="Off.TButton")
        self.projector_toggle_button.grid(row=3, column=2, padx=5, pady=5)
        self.projectors_on = False

        ttk.Button(start_controls_frame, text="End Capture", command=self.end_capture).grid(row=3, column=3, padx=5,
                                                                                            pady=5)
        self.save_one_button = ttk.Button(start_controls_frame, text="Save One (all devices)", command=self.start_save_one, state="disabled", style="Start.TButton")
        self.save_one_button.grid(row=4, column=0, padx=5, pady=5)

        row += 1

        # === Configuration Editor ===
        config_frame = ttk.LabelFrame(main_frame, text="Configuration Editor", padding=(10, 5))
        config_frame.grid(row=row, column=0, columnspan=3, sticky="ew", pady=(10, 5))
        
        # Config editor controls
        config_controls_frame = ttk.Frame(config_frame)
        config_controls_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        
        self.edit_config_button = ttk.Button(config_controls_frame, text="Edit Config", command=self.open_config_editor)
        self.edit_config_button.grid(row=0, column=0, padx=(0, 5))
        
        # Config status
        self.config_status_var = tk.StringVar(value="Config loaded")
        ttk.Label(config_controls_frame, textvariable=self.config_status_var, foreground="blue").grid(row=0, column=4, padx=(10, 0))

        row += 1

        # === Device Statuses ===
        for device, port in self.device_ports.items():
            # Get note for this device
            note = devices_config.get(str(port), {}).get("note", "")
            device_label_text = f"{device} (Port {port})"
            if note:
                device_label_text += f" - {note}"
            
            device_label = ttk.Label(main_frame, text=device_label_text)
            device_label.grid(row=row, column=0, sticky="w")
            self.device_labels[device] = device_label
            
            label = ttk.Label(main_frame, textvariable=self.status_vars[device])
            label.grid(row=row, column=1, sticky="w")
            self.status_labels[device] = label

            restart_btn = ttk.Button(main_frame, text="Restart", command=lambda p=port: self.restart_device(p))
            restart_btn.grid(row=row, column=2, padx=5)
            restart_btn.grid_remove()
            self.restart_buttons[device] = restart_btn

            row += 1

        # === Final Row: Status Message ===
        ttk.Label(main_frame, textvariable=self.message_var, foreground="blue").grid(row=row, column=0, sticky="w",
                                                                                     pady=(10, 0))

    def poll_statuses(self):
        for device, port in self.device_ports.items():
            threading.Thread(target=self.update_status, args=(device, port), daemon=True).start()
        self.check_launch_ready()  # Always call to update button states
        self.root.after(200, self.poll_statuses)

    def check_launch_ready(self):
        self.all_ready = all(self.status_vars[device].get().lower() in ["ready", "projector on", "projector off"] for device in self.device_ports)
        devices_launched = any(self.status_vars[device].get().lower() in ["ready", "projector on", "projector off", "capturing", "interrupted"] for device in self.device_ports)

        # Manage capture buttons based on device readiness
        if self.all_ready and not self.running:
            self.start_sequence_button.config(state="enabled")
            self.simple_sequence_button.config(state="enabled")
            self.save_one_button.config(state="enabled")
            # Only show "Devices Ready" if no capture is running
            self.set_message("Devices Ready!")
        elif not self.all_ready:
            self.start_sequence_button.config(state="disabled")
            self.simple_sequence_button.config(state="disabled")
            self.save_one_button.config(state="disabled")
        elif self.running:
            self.start_sequence_button.config(state="disabled")
            self.simple_sequence_button.config(state="disabled")
            self.save_one_button.config(state="disabled")
            # Don't change message when capture is running - let it show current cycle
        
        # Manage timing fields - only disable when capture is running
        if self.running:
            self.projector_sleep_entry.config(state="disabled")
            self.capture_sleep_entry.config(state="disabled")
            self.cycle_wait_entry.config(state="disabled")
        else:
            self.projector_sleep_entry.config(state="normal")
            self.capture_sleep_entry.config(state="normal")
            self.cycle_wait_entry.config(state="normal")
        
        # Edit config button: enabled unless devices are launched
        if devices_launched:
            self.edit_config_button.config(state="disabled")
        else:
            self.edit_config_button.config(state="enabled")

    def update_status(self, device, port):
        response = send_command(port, "status")
        status = response.get("status", "unknown")
        self.status_vars[device].set(status)

        # ➕ Log status history
        self.status_history[device].append(status)
        # print(f"[{device}] Status update: {status}")  # Optional logging to console

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

    def get_count(self, port):
        response = send_command(port, "count")
        status = response.get("count", 0)
        return int(status)

    def send_capture_name(self, port, name):
        try:
            socket = context.socket(zmq.REQ)
            socket.setsockopt(zmq.RCVTIMEO, 1000)
            socket.setsockopt(zmq.LINGER, 0)
            socket.connect(f"tcp://localhost:{port}")
            socket.send_json({"cmd": "set_capture_name", "name": name})
            response = socket.recv_json()
            return response
        except Exception as e:
            return {"status": "error", "detail": str(e)}
        finally:
            socket.close()


    def start_sequence(self):
        for device, port in self.device_ports.items():
            send_command(port, "capturing_off")
            send_command(port, "projector_off")
        self.set_message("Turning off all projectors...")
        time.sleep(0.5)

        if not self.all_ready:
            return

        self.start_sequence_button.config(state="disabled")
        self.simple_sequence_button.config(state="disabled")
        self.save_one_button.config(state="disabled")
        self.set_message("Capturing...")
        self.running = True
        threading.Thread(target=self._run_loop_sequence, daemon=True).start()

    def _run_loop_sequence(self):
        try:
            max_captures = self.max_captures_var.get()
            max_captures = int(max_captures) if max_captures.strip().isdigit() else None
        except Exception:
            max_captures = None

        try:
            max_cycles = self.max_cycles_var.get()
            max_cycles = int(max_cycles) if max_cycles.strip().isdigit() else None
        except Exception:
            max_cycles = None

        # Get timing configuration
        projector_sleep = float(self.projector_sleep_var.get() or "5")
        capture_sleep = float(self.capture_sleep_var.get() or "2")
        cycle_wait = float(self.cycle_wait_var.get() or "3")

        for device, port in self.device_ports.items():
            self.send_capture_name(port, self.get_current_capture_name())
            send_command(port, "inicialize")

        counts = {port:0 for device, port in self.device_ports.items()}
        cycle_count = 0
        
        while self.running:
            cycle_count += 1
            self.current_cycle = cycle_count
            self.set_message(f"Cycle {cycle_count} - Capturing...")
            
            # Check if we should stop before starting this cycle
            if not self.running:
                break
                
            for device, port in self.device_ports.items():
                send_command(port, "capturing_on")
            time.sleep(1)

            # Check if we should stop after first capture
            if not self.running:
                for device, port in self.device_ports.items():
                    send_command(port, "capturing_off")
                break

            for device, port in self.device_ports.items():
                send_command(port, "capturing_off")

            # Check if we should stop before projector sequence
            if not self.running:
                break

            for device, port in self.device_ports.items():
                send_command(port, "projector_on")
                time.sleep(projector_sleep)
                
                # Check if we should stop during projector sequence
                if not self.running:
                    send_command(port, "projector_off")
                    break
                    
                send_command(port, "capturing_on")
                time.sleep(capture_sleep)
                send_command(port, "capturing_off")
                send_command(port, "projector_off")
            
            # Check if we should stop before final wait
            if not self.running:
                break
                
            time.sleep(cycle_wait)

            for device, port in self.device_ports.items():
                counts[port] = self.get_count(port)
            print(f"Cycle {cycle_count} - Counts: {counts}")
            
            # Check if max captures reached
            if max_captures is not None and all(count >= max_captures for count in counts.values()):
                self.set_message(f"Max captures ({max_captures}) reached after {cycle_count} cycles")
                self.running = False
                break
            
            # Check if max cycles reached
            if max_cycles is not None and cycle_count >= max_cycles:
                self.set_message(f"Max cycles ({max_cycles}) completed")
                self.running = False
                break

        self.running = False
        self.current_cycle = 0

        # Ensure all devices are properly stopped
        for device, port in self.device_ports.items():
            send_command(port, "capturing_off")
            send_command(port, "projector_off")
            send_command(port, "cleanup")

        self.set_message("Capture ended, devices are still running.")
        
        # Re-enable buttons if devices are ready
        self.check_launch_ready()

    def start_simple_sequence(self):
        if not self.all_ready:
            print("not all ready")
            return
        self.start_sequence_button.config(state="disabled")
        self.simple_sequence_button.config(state="disabled")
        self.save_one_button.config(state="disabled")
        self.set_message("Starting simple capture...")
        self.running = True
        threading.Thread(target=self._run_simple_sequence, daemon=True).start()

    def _run_simple_sequence(self):
        try:
            max_captures = self.max_captures_var.get()
            max_captures = int(max_captures) if max_captures.strip().isdigit() else None
        except Exception:
            max_captures = None

        for device, port in self.device_ports.items():
            self.send_capture_name(port, self.get_current_capture_name())
            time.sleep(0.5)
            send_command(port, "inicialize")
            time.sleep(0.5)

        for device, port in self.device_ports.items():
            send_command(port, "capturing_on")

        counts = {port:0 for device, port in self.device_ports.items()}
        while self.running:
            for device, port in self.device_ports.items():
                counts[port] = self.get_count(port)
            if max_captures is not None and all(count >= max_captures for count in counts.values()):
                self.running = False
                break
            time.sleep(0.5)

        self.running = False

        for device, port in self.device_ports.items():  # turn of explicitly just to be sure
            send_command(port, "capturing_off")
        
        # Re-enable buttons if devices are ready
        self.check_launch_ready()


    def toggle_projectors(self):
        cmd = "projector_on" if not self.projectors_on else "projector_off"
        for device, port in self.device_ports.items():
            send_command(port, cmd)
            self.status_vars[device].set("Projector ON" if not self.projectors_on else "Projector OFF")

        self.projectors_on = not self.projectors_on
        state_text = "Projector ON" if self.projectors_on else "Projector OFF"
        self.projector_toggle_button.config(text=state_text)

        # Optional color indication
        if self.projectors_on:
            self.projector_toggle_button.config(style="On.TButton")
        else:
            self.projector_toggle_button.config(style="Off.TButton")

        self.set_message(f"{'ON' if self.projectors_on else 'OFF'}")

    def start_save_one(self):
        if not self.all_ready:
            self.set_message("Devices not ready")
            return

        if self.running:
            self.set_message("Another capture is already running")
            return

        # Mirror your other flows
        self.start_sequence_button.config(state="disabled")
        self.simple_sequence_button.config(state="disabled")
        self.save_one_button.config(state="disabled")
        self.set_message("Saving one frame per stream...")
        self.running = True

        threading.Thread(target=self._run_save_one, daemon=True).start()

    def _run_save_one(self):
        try:
            # Get timing configuration
            projector_sleep = float(self.projector_sleep_var.get() or "5")
                
            if not self.save_one_inicialized:
                self.set_message("Inicializing...")
                for device, port in self.device_ports.items():
                    self.send_capture_name(port, self.get_current_capture_name())
                    send_command(port, "inicialize")
                self.save_one_inicialized = True

            self.set_message("Saving projector OFF...")
            for device, port in self.device_ports.items():
                send_command(port, "save_one")
            time.sleep(1)

            self.set_message("Saving projector ON...")
            for device, port in self.device_ports.items():
                send_command(port, "projector_on")
                time.sleep(projector_sleep)
                send_command(port, "save_one")
                time.sleep(1)
                send_command(port, "projector_off")

            self.set_message("Saved! Click again to continue")

        except Exception as e:
            self.set_message(f"Save-one error: {e}")

        finally:
            self.running = False
            # Re-enable buttons if devices are ready
            self.check_launch_ready()

    def exit_devices(self):
        self.running = False
        if not self.running:
            threading.Thread(target=self._finalize_exit, daemon=True).start()

        self.set_message("Exiting")

    def end_capture(self):
        if not self.running:
            self.set_message("No capture running")
            return
            
        self.set_message(f"Stopping capture... (will finish current cycle {self.current_cycle})")
        self.running = False  # This will cause the capture loop to exit gracefully
        self.save_one_inicialized = False
        
        # Don't call cleanup immediately - let the capture loop finish gracefully
        # The cleanup will be called in the capture loop when it exits

    def _finalize_exit(self):
        self.end_capture()
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
            *python_path, capture_script,
            config["settings"], capture_name,
            "--ip", config["ip"],
            "--port", str(port)
        ]
        print(f"Restarting device on port {port} with IP {config['ip']}")
        kill_process_on_port(int(port))
        subprocess.Popen(args)

    def get_current_capture_name(self):
        capture_name = self.capture_name_var.get()
        if " " in capture_name:
            print("Capture name must not contain spaces.")
            self.set_message("Capture name must not contain spaces.")
            return
        return capture_name

    def update_device_labels(self):
        """Update device labels to show notes"""
        for device, port in self.device_ports.items():
            note = devices_config.get(str(port), {}).get("note", "")
            device_label_text = f"{device} (Port {port})"
            if note:
                device_label_text += f" - {note}"
            
            if device in self.device_labels:
                self.device_labels[device].config(text=device_label_text)

    def refresh_device_display(self):
        """Refresh the device display to match current configuration"""
        # Reload configuration from file
        global devices_config
        try:
            with open(CONFIG_FILE) as f:
                devices_config = json.load(f)
        except Exception as e:
            messagebox.showerror("Error", f"Failed to reload configuration: {str(e)}")
            return
        
        # Update device ports mapping
        self.device_ports = {f"Device {i+1}": int(port) for i, port in enumerate(devices_config.keys())}
        
        # Recreate device status variables
        self.status_vars = {name: tk.StringVar(value="Disconnected") for name in self.device_ports}
        self.status_history = {device: [] for device in self.device_ports}
        
        # Clear old device widgets
        for widget in list(self.device_labels.values()) + list(self.status_labels.values()) + list(self.restart_buttons.values()):
            if widget.winfo_exists():
                widget.destroy()
        
        # Clear dictionaries
        self.device_labels.clear()
        self.status_labels.clear()
        self.restart_buttons.clear()
        
        # Find main frame and rebuild device section
        main_frame = None
        for child in self.root.winfo_children():
            if isinstance(child, ttk.Frame):
                main_frame = child
                break
        
        if main_frame:
            # Find the row after configuration editor
            config_row = None
            for child in main_frame.grid_slaves():
                if hasattr(child, 'grid_info'):
                    grid_info = child.grid_info()
                    if hasattr(child, 'cget') and child.cget('text') == 'Configuration Editor':
                        config_row = grid_info.get('row', 0)
                        break
            
            if config_row is not None:
                device_row = config_row + 1
                
                # Rebuild device status display
                for device, port in self.device_ports.items():
                    # Get note for this device
                    note = devices_config.get(str(port), {}).get("note", "")
                    device_label_text = f"{device} (Port {port})"
                    if note:
                        device_label_text += f" - {note}"
                    
                    device_label = ttk.Label(main_frame, text=device_label_text)
                    device_label.grid(row=device_row, column=0, sticky="w")
                    self.device_labels[device] = device_label
                    
                    label = ttk.Label(main_frame, textvariable=self.status_vars[device])
                    label.grid(row=device_row, column=1, sticky="w")
                    self.status_labels[device] = label

                    restart_btn = ttk.Button(main_frame, text="Restart", command=lambda p=port: self.restart_device(p))
                    restart_btn.grid(row=device_row, column=2, padx=5)
                    restart_btn.grid_remove()
                    self.restart_buttons[device] = restart_btn

                    device_row += 1
        
        self.set_message("Device display refreshed")
        self.config_status_var.set("Display refreshed")

    # Configuration Editor Methods
    def validate_ip_address(self, ip):
        """Validate IP address format"""
        pattern = r'^(\d{1,3}\.){3}\d{1,3}$'
        if not re.match(pattern, ip):
            return False
        parts = ip.split('.')
        return all(0 <= int(part) <= 255 for part in parts)

    def validate_port(self, port):
        """Validate port number"""
        try:
            port_num = int(port)
            return 1024 <= port_num <= 65535
        except ValueError:
            return False

    def open_config_editor(self):
        """Open configuration editor window"""
        # Check if devices are launched
        devices_launched = any(self.status_vars[device].get().lower() in ["ready", "projector on", "projector off", "capturing", "interrupted"] for device in self.device_ports)
        
        if devices_launched:
            messagebox.showwarning("Configuration Locked", 
                                 "Cannot edit configuration while devices are launched.\n"
                                 "Please exit devices before editing configuration.")
            return
        
        editor_window = tk.Toplevel(self.root)
        editor_window.title("Configuration Editor")
        editor_window.geometry("600x500")
        editor_window.transient(self.root)
        editor_window.grab_set()

        # Create main frame with scrollbar
        main_frame = ttk.Frame(editor_window)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        # Create canvas and scrollbar for scrolling
        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Store entries for this editor session
        session_entries = {}

        # Create entries for each device
        row = 0
        for port, config in devices_config.items():
            device_frame = ttk.LabelFrame(scrollable_frame, text=f"Device on Port {port}", padding=10)
            device_frame.grid(row=row, column=0, columnspan=2, sticky="ew", pady=5)

            # IP Address
            ttk.Label(device_frame, text="IP Address:").grid(row=0, column=0, sticky="w", padx=(0, 10))
            ip_var = tk.StringVar(value=config["ip"])
            ip_entry = ttk.Entry(device_frame, textvariable=ip_var, width=20)
            ip_entry.grid(row=0, column=1, sticky="w", padx=(0, 20))
            session_entries[f"{port}_ip"] = ip_var

            # Settings
            ttk.Label(device_frame, text="Settings:").grid(row=0, column=2, sticky="w", padx=(0, 10))
            settings_var = tk.StringVar(value=config["settings"])
            settings_entry = ttk.Entry(device_frame, textvariable=settings_var, width=15)
            settings_entry.grid(row=0, column=3, sticky="w")

            # Note
            ttk.Label(device_frame, text="Note:").grid(row=1, column=0, sticky="w", padx=(0, 10))
            note_var = tk.StringVar(value=config.get("note", ""))
            note_entry = ttk.Entry(device_frame, textvariable=note_var, width=30)
            note_entry.grid(row=1, column=1, columnspan=2, sticky="w", padx=(0, 20))
            session_entries[f"{port}_note"] = note_var

            # Port (read-only, for reference)
            ttk.Label(device_frame, text="Port:").grid(row=2, column=0, sticky="w", padx=(0, 10))
            ttk.Label(device_frame, text=port, foreground="gray").grid(row=2, column=1, sticky="w", padx=(0, 20))

            # Remove device button
            ttk.Button(device_frame, text="Remove Device", 
                      command=lambda p=port: self.remove_device_confirm(p, editor_window, session_entries)).grid(row=2, column=2, columnspan=2, sticky="e", padx=(10, 0))

            session_entries[f"{port}_settings"] = settings_var
            row += 1

        # Buttons frame
        buttons_frame = ttk.Frame(scrollable_frame)
        buttons_frame.grid(row=row, column=0, columnspan=2, pady=20)

        ttk.Button(buttons_frame, text="Apply Changes", 
                  command=lambda: self.apply_config_changes(session_entries, editor_window)).grid(row=0, column=0, padx=5)
        ttk.Button(buttons_frame, text="Cancel", 
                  command=editor_window.destroy).grid(row=0, column=1, padx=5)
        ttk.Button(buttons_frame, text="Add New Device", 
                  command=lambda: self.add_new_device_editor(editor_window, session_entries)).grid(row=0, column=2, padx=5)

        # Pack canvas and scrollbar
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Bind mousewheel to canvas
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1*(event.delta/120)), "units")
        
        def _on_mousewheel_linux(event):
            if event.num == 4:
                canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                canvas.yview_scroll(1, "units")
        
        # Bind for Windows/Mac
        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        # Bind for Linux
        canvas.bind_all("<Button-4>", _on_mousewheel_linux)
        canvas.bind_all("<Button-5>", _on_mousewheel_linux)

        def _unbind_mousewheel(event):
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Button-4>")
            canvas.unbind_all("<Button-5>")
        editor_window.bind("<Destroy>", _unbind_mousewheel)

    def add_new_device_editor(self, parent_window, session_entries):
        """Add a new device to the configuration"""
        dialog = tk.Toplevel(parent_window)
        dialog.title("Add New Device")
        dialog.geometry("400x200")
        dialog.transient(parent_window)
        dialog.grab_set()

        ttk.Label(dialog, text="Port:").grid(row=0, column=0, sticky="w", padx=10, pady=5)
        port_var = tk.StringVar()
        ttk.Entry(dialog, textvariable=port_var, width=15).grid(row=0, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(dialog, text="IP Address:").grid(row=1, column=0, sticky="w", padx=10, pady=5)
        ip_var = tk.StringVar()
        ttk.Entry(dialog, textvariable=ip_var, width=20).grid(row=1, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(dialog, text="Settings:").grid(row=2, column=0, sticky="w", padx=10, pady=5)
        settings_var = tk.StringVar(value="dai3")
        ttk.Entry(dialog, textvariable=settings_var, width=15).grid(row=2, column=1, sticky="w", padx=10, pady=5)

        ttk.Label(dialog, text="Note:").grid(row=3, column=0, sticky="w", padx=10, pady=5)
        note_var = tk.StringVar()
        ttk.Entry(dialog, textvariable=note_var, width=20).grid(row=3, column=1, sticky="w", padx=10, pady=5)

        def add_device():
            port = port_var.get().strip()
            ip = ip_var.get().strip()
            settings = settings_var.get().strip()
            note = note_var.get().strip()

            if not port or not ip or not settings:
                messagebox.showerror("Error", "Port, IP, and Settings are required")
                return

            if not self.validate_port(port):
                messagebox.showerror("Error", "Invalid port number (1024-65535)")
                return

            if not self.validate_ip_address(ip):
                messagebox.showerror("Error", "Invalid IP address format")
                return

            if port in devices_config:
                messagebox.showerror("Error", f"Port {port} already exists")
                return

            # Add to current session
            session_entries[f"{port}_ip"] = tk.StringVar(value=ip)
            session_entries[f"{port}_settings"] = tk.StringVar(value=settings)
            session_entries[f"{port}_note"] = tk.StringVar(value=note)
            
            # Add to devices_config
            devices_config[port] = {"ip": ip, "settings": settings, "note": note}
            
            dialog.destroy()
            parent_window.destroy()
            self.open_config_editor()  # Refresh editor

        ttk.Button(dialog, text="Add Device", command=add_device).grid(row=4, column=0, padx=10, pady=20)
        ttk.Button(dialog, text="Cancel", command=dialog.destroy).grid(row=4, column=1, padx=10, pady=20)

    def remove_device_confirm(self, port, editor_window, session_entries):
        """Confirm and remove a device from the configuration"""
        device_info = devices_config.get(port, {})
        ip = device_info.get("ip", "Unknown")
        note = device_info.get("note", "")
        
        # Create confirmation message
        confirm_msg = f"Are you sure you want to remove this device?\n\nPort: {port}\nIP: {ip}"
        if note:
            confirm_msg += f"\nNote: {note}"
        
        if messagebox.askyesno("Confirm Device Removal", confirm_msg):
            self.remove_device(port, editor_window, session_entries)

    def remove_device(self, port, editor_window, session_entries):
        """Remove a device from the configuration"""
        try:
            # Remove from devices_config
            if port in devices_config:
                del devices_config[port]
            
            # Remove from session_entries
            keys_to_remove = [key for key in session_entries.keys() if key.startswith(f"{port}_")]
            for key in keys_to_remove:
                del session_entries[key]
            
            # Close and reopen the editor to refresh the display
            editor_window.destroy()
            self.open_config_editor()
            
            messagebox.showinfo("Success", f"Device on port {port} has been removed successfully!")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to remove device: {str(e)}")

    def apply_config_changes(self, session_entries, editor_window):
        """Apply configuration changes from the editor and save to file"""
        try:
            # Validate all entries
            for key, var in session_entries.items():
                if key.endswith("_ip"):
                    if not self.validate_ip_address(var.get()):
                        messagebox.showerror("Error", f"Invalid IP address: {var.get()}")
                        return
                elif key.endswith("_settings"):
                    if not var.get().strip():
                        messagebox.showerror("Error", "Settings cannot be empty")
                        return

            # Apply changes
            for key, var in session_entries.items():
                if key.endswith("_ip"):
                    port = key.replace("_ip", "")
                    devices_config[port]["ip"] = var.get().strip()
                elif key.endswith("_settings"):
                    port = key.replace("_settings", "")
                    devices_config[port]["settings"] = var.get().strip()
                elif key.endswith("_note"):
                    port = key.replace("_note", "")
                    devices_config[port]["note"] = var.get().strip()

            # Save to file automatically
            self.save_config_to_file()
            
            # Update device ports mapping
            self.device_ports = {f"Device {i+1}": int(port) for i, port in enumerate(devices_config.keys())}
            
            # Refresh the device display to reflect config changes
            self.refresh_device_display()
            
            editor_window.destroy()
            messagebox.showinfo("Success", "Configuration changes applied and saved successfully!")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to apply changes: {str(e)}")

    def save_config_to_file(self):
        """Save current configuration to file (internal method)"""
        try:
            # Create backup
            backup_file = CONFIG_FILE + f".backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            shutil.copy2(CONFIG_FILE, backup_file)
            
            # Save current config
            with open(CONFIG_FILE, 'w') as f:
                json.dump(devices_config, f, indent=2)
            
            self.original_config = devices_config.copy()
            self.config_modified = False
            self.config_status_var.set("Config saved")
            self.set_message("Configuration saved successfully")
            
        except Exception as e:
            raise Exception(f"Failed to save configuration: {str(e)}")

    def save_config(self):
        """Save current configuration to file (public method)"""
        try:
            self.save_config_to_file()
        except Exception as e:
            messagebox.showerror("Error", str(e))



    def launch_all_devices(self):
        self.set_message("Launching devices...")


        self.projector_toggle_button.config(text="Projector OFF")
        self.projector_toggle_button.config(style="Off.TButton")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        capture_name = self.get_current_capture_name()
        root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

        for port, config in devices_config.items():
            args = [
                *python_path, capture_script,
                config["settings"], capture_name,
                "--ip", config["ip"],
                "--port", str(port),
                "--output", root_path,
            ]
            print(f"[Launch] Launching device on port {port} with args: {args}")
            try:
                kill_process_on_port(int(port))
                subprocess.Popen(args)
            except Exception as e:
                print(f"[ERROR] Failed to launch on port {port}: {e}")


# ----- Run GUI -----
if __name__ == "__main__":
    root = tk.Tk()
    app = MultiDeviceControlApp(root)
    root.mainloop()
