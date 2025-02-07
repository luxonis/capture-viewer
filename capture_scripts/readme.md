# UNIVERSAL CAPTURE

You can now run capture for OAKs stereo and OAK-D-SR-POE using one script.

```bash
python capture_scripts/oak_capture_universal.py default 3_cameras --devices 19443010A15BA12E00 1944301081DE992E00 194430104100A22E00
```

Arguments:
- **settings** - choose `default`, `default_tof` or add path to your own settings.
- **view_name** - the name of capture that will be saved in metadata file
- **--output** (optional) - path to custom output folder, where the output of capture will be created. By default, it will be saved in DATA folder in capture-viewer
- **--autostart** (optional) - number of frames the script will droped before automatically starting the capture
- **--devices** (optional) - the mxids of devices to connnect to. If left blank, it will connect to the first available device.

## Multiple TOF capture

For multiple TOF capture you need to configure the appropriate settings json as:
- turn ON RIGHT stream and turn OFF LEFT stream
- turn OFF sync node - that configures software synchronization which might interfere with fsync
- Connect the cameras with FSYNC cables

Appropriate settings are saved in `tof_multiple.json`

If the devices are OAK D SR POE and connected with fsync, the script will automatically
configure the appropriate fsync settings. Trying to run multiple TOFs without fsync cable connected might cause problems.

```bash
python capture_scripts/oak_capture_universal.py <appropriate_tof_config> 3_cameras --devices <mxids_of_tofs>
```