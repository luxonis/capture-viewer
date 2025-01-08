JSON_CONFIG=default.json
#JSON_CONFIG=default_irOff.json
AUTOSTART=100

python capture_scripts/oak_capture.py settings_jsons/${JSON_CONFIG} LEFT --autostart ${AUTOSTART} --device-ip "19443010B1D99A2E00" &
python capture_scripts/oak_capture.py settings_jsons/${JSON_CONFIG} CENTER --autostart ${AUTOSTART} --device-ip "18443010F175870E00" &
python capture_scripts/oak_capture.py settings_jsons/${JSON_CONFIG} RIGHT --autostart ${AUTOSTART} --device-ip "1944301021AA992E00" &