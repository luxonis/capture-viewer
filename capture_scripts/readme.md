# Captures additional info

## Multiple TOF capture

For multiple TOF capture run `sr_poe_capture_multiple_fsync.py` you need to configure the appropriate settings json as:
- turn ON RIGHT stream and turn OFF LEFT stream
- turn of sync node - that configures software synchronization which might interfere with fsync
- Connect the cameras with FSYNC cables

To have synced TOF sensors, you need to always run the RIGHT stream, as the RIGHT camera is the source of the fsync signal

example usage:

```bash
 python capture_scripts/sr_poe_capture_multiple_fsync.py /home/katka/PycharmProjects/capture-viewer/settings_jsons/sr_poe_settings_default.json view --devices 14442C1091F5D9E700 14442C10F10AC8D600
```


This universal script works for OAKs stereo and OAK-D-SR-POE

without devices it will just connect the first available device

```bash
python capture_scripts/oak_capture_universal.py default 3_cameras --devices 19443010A15BA12E00 1944301081DE992E00 194430104100A22E00
```