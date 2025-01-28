# Captures additional info

## Multiple TOF capture

For multiple TOF capture run `sr_poe_capture_multiple_fsync.py` you need to configure the appropriate settings json as:
- turn ON RIGHT stream and turn OFF LEFT stream
- turn of sync node - that configures software synchronization which might interfere with fsync

To have synced TOF sensors, you need to always run the RIGHT stream, as the RIGHT camera is the source of the fsync signal