Raspberry Pi YOLO INT8 deployment
==================================

Target:
- Raspberry Pi 4B
- Raspbian / Raspberry Pi OS Buster 10
- 32-bit armv7l
- USB YUYV camera at /dev/video0

Contents:
- YOLO26n 320 NCNN full INT8 model
- Stable Cortex-A72/NEON/OpenMP/LTO binary
- JSON Lines output for motor integration
- Python non-blocking VisionClient API
- Optional browser MJPEG preview

Install on the Raspberry Pi:

    chmod +x install.sh
    sudo ./install.sh

Background JSON mode for the motor controller:

    rpi-yolo

20-frame camera check:

    rpi-yolo --max-frames 20

Browser preview:

    rpi-yolo-preview

Then open:

    http://RASPBERRY_PI_IP:8080/

Python motor-side API:

    from rpi_yolo_api import VisionClient

    with VisionClient() as vision:
        control = vision.get_control(max_age_ms=700)

The motor loop must check control["valid"] before acting.
See vision_client_example.py for a complete polling loop.

Notes:
- This is the full INT8 model. It is faster than the FP16 package but may
  have lower distant-target accuracy.
- Do not run rpi-yolo and rpi-yolo-preview at the same time. Both own the
  camera device.
- Browser preview adds JPEG/network overhead. Do not enable it in the final
  motor-control process.
- The installer does not configure motors, infrared sensors, GPIO, or
  automatic startup.
