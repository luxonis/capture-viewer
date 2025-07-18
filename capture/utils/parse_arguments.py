import depthai as dai
import argparse
import datetime
import os


def parseArguments(root_path):
    # PARSE ARGUMENTS
    parser = argparse.ArgumentParser()
    # Mandatory arguments
    parser.add_argument("settings_file_path", help="Path to settings JSON")
    parser.add_argument("view_name", help="Name of the capture")
    parser.add_argument("--output", default=root_path, help="Custom output folder")
    parser.add_argument("--autostart", default=-1, type=int, help='Automatically start capturing after given number of seconds (-1 to disable)')
    # parser.add_argument("--devices", default=[], dest="mxids", nargs="+", help="MXIDS of devices to connect to")
    parser.add_argument("--ip", default=None, dest="ip", help="IP to connect to")
    parser.add_argument("--autostart_time", default=0, help="Select a fixed time when the script is supposed to start")
    parser.add_argument("--autostart_end", default=0, help="Select a fixed time for capture to end")
    parser.add_argument("--show_streams", default=False, help="Show all the running streams. If false, only shows the left frame")
    parser.add_argument("--alternating_capture", default=False, help="Turn ON/OFF IR projector")

    return parser.parse_args()

def process_argument_logic(args):
    settings_path = args.settings_file_path
    view_name = args.view_name
    ip = args.ip

    # SETTINGS loading
    if not os.path.exists(settings_path):
        settings_path_1 = f"settings_jsons/{settings_path}.json"
        settings_path_2 = f"settings_jsons/{settings_path}"
        if os.path.exists(settings_path_1):
            settings_path = settings_path_1
        elif os.path.exists(settings_path_2):
            settings_path = settings_path_2
        else: raise FileNotFoundError(f"Settings file '{settings_path}' does not exist.")

    today = datetime.date.today()

    if args.autostart_time:
        wait = datetime.datetime.combine(today, datetime.time.fromisoformat(args.autostart_time))
    else:
        wait = 0

    if args.autostart_end:
        wait_end = datetime.datetime.combine(today, datetime.time.fromisoformat(args.autostart_end))
    else:
        wait_end = 0

    if args.autostart_time: args.autostart = 0

    return settings_path, view_name, ip, args.autostart, wait, wait_end, args.show_streams, args.alternating_capture