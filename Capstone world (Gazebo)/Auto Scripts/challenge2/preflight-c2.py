#!/bin/bash

import os
import importlib.util
import sys

GREEN = "\033[92m"
RED = "\033[91m"
RESET = "\033[0m"

# need SERIAL FOR c2
# Need 3.9 or below python
# check usb.

# check permissions 666 /dev/ttyACM0

files_to_check = [
    'aruco_library.py',      
    'calibration_logi.npz',
    'challenge2.py',
    'Logs/',                 
    'preflight-c2.py',
    'latch.py'
]

# Check that Python 3.9 or below is installed
def check_python_version():
    version = sys.version_info
    if version.major != 3 or version.minor > 9:
        print(f"{RED}✗{RESET} P2ython 3.9 or below is required (detected {version.major}.{version.minor}.{version.micro}).")
        return False
    else:
        print(f"{GREEN}✓{RESET} Python {version.major}.{version.minor}.{version.micro} is acceptable.")
        return True


def check_files():
    print("File checks:")
    all_exist = True
    for file in files_to_check:
        if os.path.exists(file):
            print(f"{GREEN}✓{RESET} {file} exists.")
        else:
            print(f"{RED}✗{RESET} {file} does not exist.")
            all_exist = False
    return all_exist

def check_packages():
    print("\nPackage checks:")
    packages_to_check = [
        'numpy',
        'cv2',
        'dronekit',  # for Ardupilot communication
        'pymavlink', # MAVLink support
        # ASS NEEDED
        # 'Jetson.GPIO',  # Jetson-specific GPIO control or 
        'RPi.GPIO'     # OR this. THIS IS FOR RPi, I believe we are going with pie so keep.
    ]
    all_installed = True
    for package in packages_to_check:
        if importlib.util.find_spec(package) is not None:
            print(f"{GREEN}✓{RESET} Package '{package}' is installed.")
        else:
            print(f"{RED}✗{RESET} Package '{package}' is missing.")
            all_installed = False
    return all_installed

def check_permissions():
    print("\nPermission checks:")
    devices = {
        'Flight Controller': '/dev/ttyACM0',
        'Telemetry': '/dev/ttyUSB0'
    }
    all_permissions_ok = True

    for device_name, device_path in devices.items():
        try:
            mode = os.stat(device_path).st_mode
            if oct(mode)[-3:] == '666':
                print(f"{GREEN}✓{RESET} {device_name} ({device_path}) has correct permissions (666).")
            else:
                print(f"{RED}✗{RESET} {device_name} ({device_path}) does not have correct permissions (expected 666, found {oct(mode)[-3:]}).")
                all_permissions_ok = False
        except FileNotFoundError:
            print(f"{RED}✗{RESET} {device_name} ({device_path}) does not exist.")
            all_permissions_ok = False

    return all_permissions_ok

def main():
    version_ok = check_python_version()
    files_ok = check_files()
    packages_ok = check_packages()
    permissions_ok = check_permissions()
    if version_ok and files_ok and packages_ok and permissions_ok:
        print(f"\n{GREEN}All pre-flight checks passed. Ready to run challenge1.py!{RESET}")
    else:
        print(f"\n{RED}Pre-flight checks failed.{RESET}")

if __name__ == '__main__':
    main()