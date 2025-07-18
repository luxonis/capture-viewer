

def parseArguments():
    # PARSE ARGUMENTS
    parser = argparse.ArgumentParser()
    # Mandatory arguments
    parser.add_argument("settings_file_path", help="Path to settings JSON")
    parser.add_argument("view_name", help="What part of the scene the camera is looking at")
    # Optional argument with a flag for device_ip
    parser.add_argument("--device-ip", dest="device_ip", help="IP of remote device", default=None)
    parser.add_argument("--autostart", default=-1, type=int, help='Automatically start capturing after given number of frames (-1 to disable)')

    args = parser.parse_args()
    settings_path = args.settings_file_path
    view_name = args.view_name
    device_info = None
    if args.device_ip:
        device_info = dai.DeviceInfo(args.device_ip)

    # SETTINGS loading
    if not os.path.exists(settings_path):
        settings_path_1 = f"settings_jsons/{settings_path}.json"
        settings_path_2 = f"settings_jsons/{settings_path}"
        if os.path.exists(settings_path_1):
            settings_path = settings_path_1
        elif os.path.exists(settings_path_2):
            settings_path = settings_path_2
        else: raise FileNotFoundError(f"Settings file '{settings_path}' does not exist.")

    return settings_path, view_name, device_info, args.autostart