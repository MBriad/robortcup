"""Example motor-side use of the non-blocking vision function."""

import time

from rpi_yolo_api import VisionClient


def main():
    with VisionClient() as vision:
        while True:
            control = vision.get_control(max_age_ms=700)
            if not control["valid"]:
                print("VISION LOST: stop or enter safe search", control["reason"])
            elif control["action"] == "push":
                print("GOOD: PID error =", control["offset_x"])
            elif control["action"] == "scan_with_ir":
                print("BAD: query nearby infrared sensors")
            else:
                print("NO TARGET: search")
            time.sleep(0.02)  # Example 50 Hz motor loop; vision stays asynchronous.


if __name__ == "__main__":
    main()
