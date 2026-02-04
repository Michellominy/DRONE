import tk_tools
import threading
import time
import random
from typing import Callable, Optional


class DroneOrientation:
    """Widget that shows drone yaw/pitch/roll and can update itself from a background thread.

    Use `start_update_thread(interval, fetch_fn)` to start periodic updates. If `fetch_fn` is None
    the widget will use random values (useful for demo/testing). Updates are scheduled on the
    Tk main thread using `after` to remain thread-safe.
    """
    def __init__(self, root):                
        self.root = root
        self.yaw_meter = tk_tools.Gauge(root, min_value=-90, max_value=90,
                       label='Yaw', unit='º/s', height=150, width=200, divisions=0)
        self.yaw_meter.grid(column=0, row=0, columnspan=1, rowspan=1)
        self.pitch_meter = tk_tools.Gauge(root, min_value=-90, max_value=90,
                       label='Pitch', unit='º/s', height=150, width=200, divisions=0)
        self.pitch_meter.grid(column=1, row=0, columnspan=1, rowspan=1)
        self.roll_meter = tk_tools.Gauge(root, min_value=-90, max_value=90,
                       label='Roll', unit='º/s', height=150, width=200, divisions=0)
        self.roll_meter.grid(column=2, row=0, columnspan=1, rowspan=1)
        
        self.yaw_meter.set_value(0)
        self.pitch_meter.set_value(0)
        self.roll_meter.set_value(0)

        # Thread control
        self._update_thread: Optional[threading.Thread] = None
        self._running = False

        # Stop thread when the widget is destroyed
        try:
            root.bind('<Destroy>', lambda e: self.stop_update_thread())
        except Exception:
            pass
        
    def update_orientation(self, yaw: float, pitch: float, roll: float) -> None:
        """Update the gauge values. Must be called from the Tk main thread."""
        self.yaw_meter.set_value(yaw)
        self.pitch_meter.set_value(pitch)
        self.roll_meter.set_value(roll)

    def _schedule_update(self, yaw: float, pitch: float, roll: float) -> None:
        """Schedule update on the Tk main thread."""
        try:
            # Use root.after to safely call update on the main loop
            self.root.after(0, lambda: self.update_orientation(yaw, pitch, roll))
        except Exception:
            # If scheduling fails for any reason, ignore to keep the background loop running
            pass

    def start_update_thread(self, interval: float = 2.0, fetch_fn: Optional[Callable[[], tuple]] = None) -> None:
        """Start a background daemon thread that periodically fetches orientation and updates widgets.

        - interval: seconds between updates
        - fetch_fn: callable returning (yaw, pitch, roll). If None, random values are used.
        """
        if getattr(self, '_running', False):
            return
        self._running = True

        def _loop():
            while self._running:
                if fetch_fn:
                    try:
                        vals = fetch_fn()
                        if isinstance(vals, (list, tuple)) and len(vals) >= 3:
                            yaw, pitch, roll = vals[0], vals[1], vals[2]
                        else:
                            yaw, pitch, roll = 0.0, 0.0, 0.0
                    except Exception:
                        yaw, pitch, roll = 0.0, 0.0, 0.0
                else:
                    yaw = random.uniform(-90.0, 90.0)
                    pitch = random.uniform(-90.0, 90.0)
                    roll = random.uniform(-90.0, 90.0)

                self._schedule_update(yaw, pitch, roll)
                time.sleep(interval)

        t = threading.Thread(target=_loop, daemon=True)
        self._update_thread = t
        t.start()

    def stop_update_thread(self, timeout: float = 1.0) -> None:
        """Stop the background thread and wait up to `timeout` seconds for it to exit."""
        self._running = False
        thread = getattr(self, '_update_thread', None)
        self._update_thread = None
        if thread and thread.is_alive():
            try:
                thread.join(timeout=timeout)
            except Exception:
                pass
