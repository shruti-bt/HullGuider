#!/usr/bin/env python3

import socket
import json
import time
import sys

DVL_IP = "192.168.2.95"
DVL_PORT = 16171

FREQ = 5.0  # desired ping frequency (Hz)
PERIOD = 1.0 / FREQ


def send_command(command_dict):
    """
    Send a JSON command to the DVL and return the parsed response.
    """
    try:
        payload = json.dumps(command_dict).encode("utf-8") + b"\n"

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((DVL_IP, DVL_PORT))

        s.sendall(payload)

        response = s.recv(4096)
        s.close()

        if response:
            return json.loads(response.decode())

    except Exception as e:
        print(f"[ERROR] Command failed: {e}")

    return None


def disable_internal_pinging():
    """
    Disable the internal acoustic ping scheduler so we can externally trigger pings.
    """
    print("Disabling internal acoustic pinging...")

    cmd = {
        "command": "set_config",
        "parameters": {
            "acoustic_enabled": False
        }
    }

    resp = send_command(cmd)

    if resp and resp.get("success", False):
        print("Internal pinging disabled.")
    else:
        print("Warning: could not confirm acoustic disable.")


def trigger_ping():
    """
    Trigger one ping.
    """
    cmd = {"command": "trigger_ping"}
    return send_command(cmd)


def main():
    print(f"DVL external trigger controller")
    print(f"Target frequency: {FREQ} Hz ({PERIOD:.3f}s period)")
    print()

    disable_internal_pinging()

    print("\nStarting ping loop...\n")

    try:
        while True:

            start = time.time()

            resp = trigger_ping()

            if resp:
                if resp.get("success", False):
                    print("Ping queued")
                else:
                    print("Queue full or ping rejected")
                    time.sleep(0.2)
            else:
                print("No response from DVL")

            elapsed = time.time() - start
            sleep_time = PERIOD - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopping ping loop.")
        sys.exit(0)


if __name__ == "__main__":
    main()