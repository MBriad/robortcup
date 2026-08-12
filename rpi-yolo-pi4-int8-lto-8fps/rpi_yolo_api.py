"""Non-blocking Python interface for the C++ NCNN vision service."""

import copy
import json
import subprocess
import threading
import time


class VisionClient:
    """Keep vision running and expose its latest result to a fast motor loop."""

    def __init__(self, command=None):
        self.command = list(command or ("rpi-yolo", "--report-every", "0"))
        self._process = None
        self._thread = None
        self._lock = threading.Lock()
        self._latest = None
        self._received_at = None
        self._reader_error = None

    def start(self):
        if self._process is not None and self._process.poll() is None:
            return self
        with self._lock:
            self._latest = None
            self._received_at = None
            self._reader_error = None
        self._process = subprocess.Popen(
            self.command,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return self

    def _read_loop(self):
        try:
            for line in self._process.stdout:
                try:
                    result = json.loads(line)
                except json.JSONDecodeError:
                    continue
                with self._lock:
                    self._latest = result
                    self._received_at = time.monotonic()
        except Exception as error:
            with self._lock:
                self._reader_error = str(error)

    def get_raw(self, max_age_ms=700):
        """Return the latest fresh JSON object, or None if absent/stale."""
        with self._lock:
            if self._latest is None or self._received_at is None:
                return None
            age_ms = (time.monotonic() - self._received_at) * 1000.0
            if age_ms > max_age_ms:
                return None
            result = copy.deepcopy(self._latest)
        result["age_ms"] = age_ms
        return result

    def get_control(self, max_age_ms=700):
        """Return one stable dictionary intended for the motor controller."""
        result = self.get_raw(max_age_ms=max_age_ms)
        if result is None:
            with self._lock:
                reason = self._reader_error or "no_data_or_stale"
            return {
                "valid": False,
                "reason": reason,
                "sequence": None,
                "timestamp_ms": None,
                "frame_width": None,
                "frame_height": None,
                "action": "search",
                "target_type": None,
                "confidence": None,
                "center_x": None,
                "center_y": None,
                "offset_x": None,
                "offset_y": None,
                "distance_cm": None,
                "age_ms": None,
            }

        if result.get("status") == "error":
            return {
                "valid": False,
                "reason": result.get("error", "vision_error"),
                "sequence": result.get("sequence"),
                "timestamp_ms": result.get("timestamp_ms"),
                "frame_width": result.get("frame_width"),
                "frame_height": result.get("frame_height"),
                "action": "search",
                "target_type": None,
                "confidence": None,
                "center_x": None,
                "center_y": None,
                "offset_x": None,
                "offset_y": None,
                "distance_cm": None,
                "age_ms": result["age_ms"],
            }

        target = result.get("target")
        return {
            "valid": True,
            "reason": "ok",
            "sequence": result.get("sequence"),
            "timestamp_ms": result.get("timestamp_ms"),
            "frame_width": result.get("frame_width"),
            "frame_height": result.get("frame_height"),
            "action": result.get("action", "search"),
            "target_type": target.get("type") if target else None,
            "confidence": target.get("confidence") if target else None,
            "center_x": target.get("center_x") if target else None,
            "center_y": target.get("center_y") if target else None,
            "offset_x": target.get("offset_x") if target else None,
            "offset_y": target.get("offset_y") if target else None,
            "distance_cm": target.get("distance_cm") if target else None,
            "age_ms": result["age_ms"],
        }

    def close(self):
        process = self._process
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        if self._thread is not None:
            self._thread.join(timeout=1)
        self._process = None
        self._thread = None

    def __enter__(self):
        return self.start()

    def __exit__(self, _type, _value, _traceback):
        self.close()
