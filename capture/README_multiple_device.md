# Multi-Device Capture GUI

##  What is it?

This update introduces a **Python GUI controller** to manage multiple Luxonis DepthAI devices for synchronized capture with dot projector control.


## Where to Specify Devices
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

Use settings with raw streams enabled for RVC4 devices and without raw streams for RVC2 devices.

For running 4 devices with 2x2+2x4=12 output streams (enabling left and right on all devices plus 
raw left and right on RVC4) runs best on `5 FPS`.

## How to Run

From the `capture/` folder (or wherever the GUI script is located), run:

```bash
python capture/capture_control.py
```
 This will open a graphical interface showing all devices, their status, and control buttons.


## Requirements
```bash
pip install capture/requirements_capture.txt
```

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
