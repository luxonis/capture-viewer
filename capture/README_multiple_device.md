# Multi-Device Capture GUI

## 🧠 What is it?

This update introduces a **Python GUI controller** to manage multiple Luxonis DepthAI devices for synchronized capture with dot projector control.

It replaces manual script launching with a graphical interface and supports:

- 🔌 Viewing live connection status for each device
- 🖥️ Launching capture scripts (`dai3_port_capture.py`) via `conda run`
- 📷 Starting & stopping synchronized capture loops
- 🔄 Restarting individual devices if they disconnect
- 📣 Displaying real-time status messages


## ▶️ How to Run

From the `capture/` folder (or wherever the GUI script is located), run:

```bash
python capture/capture_control.py
```
🧪 This will open a graphical interface showing all devices, their status, and control buttons.

## 📁 Where to Specify Devices
Edit this file: `capture/capture_control_config.json`

This JSON defines the list of devices, IPs, ports, and their settings.

``` json
{
  "5555": { "ip": "192.168.50.102", "settings": "dai3" },
  "5556": { "ip": "192.168.50.103", "settings": "dai3" },
  "5557": { "ip": "192.168.50.118", "settings": "dai3_raw" },
  "5558": { "ip": "192.168.50.130", "settings": "dai3_raw" }
}
```

## ✅ Requirements
- Python packages: tkinter, zmq, subprocess


---

## Multiple TOF capture

For multiple TOF capture you need to configure the appropriate settings json as:
- turn ON RIGHT stream and turn OFF LEFT stream
- turn OFF sync node - that configures software synchronization which might interfere with fsync
- Connect the cameras with FSYNC cables

Appropriate settings are saved in `capture/settings_jsons/tof_multiple.json`

If the devices are OAK D SR POE and connected with fsync, the script will automatically
configure the appropriate fsync settings. Trying to run multiple TOFs without fsync cable connected might cause problems.

```bash
python capture/dai2_capture.py <appropriate_tof_config> <capture_name> --devices <mxids_of_tofs>
```
