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