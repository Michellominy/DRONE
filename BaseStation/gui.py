"""Quadcopter ground station. All Tk calls run on the main thread."""
import json
import math
import os
import queue
import sys
import time
import tkinter as tk

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from BaseStation.Controller import Controller
from BaseStation.widgets.drone_telemetry import DroneTelemetry
from BaseStation.widgets.logs import Logs
from BaseStation.widgets.throttles import Throttles
from common.constants import DRONE_IP
from common.zeroMQManager import ZeroMQSubscriber, zeroMQServer

BG = '#0b0f14'
PANEL = '#10151d'
BORDER = '#344252'
TEXT = '#e8f0f8'
MUTED = '#9aabbc'
GREEN = '#39d98a'
ORANGE = '#ffb74d'

telemetry_queue = queue.Queue(maxsize=100)
logs_queue = queue.Queue(maxsize=250)
controller_queue = queue.Queue(maxsize=200)


def enqueue_latest(q, item):
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
        except queue.Full:
            pass


def onHeightThrottleChange(value):
    zeroMQServer.send('throttle', {'data': value})


def onYawThrottleChange(value):
    zeroMQServer.send('yaw', {'data': value})


def onPitchThrottleChange(value):
    zeroMQServer.send('pitch', {'data': value})


def onRollThrottleChange(value):
    zeroMQServer.send('roll', {'data': value})


root = tk.Tk()
root.title('Quadcopter | Ground Station')
root.geometry('1400x900')
root.minsize(1050, 700)
root.configure(bg=BG)
root.columnconfigure(0, weight=4)
root.columnconfigure(1, weight=7)
root.rowconfigure(1, weight=4)
root.rowconfigure(2, weight=1)

header = tk.Frame(root, bg=PANEL, height=70, highlightbackground=BORDER, highlightthickness=1)
header.grid(row=0, column=0, columnspan=2, sticky='ew')
header.grid_propagate(False)
heading = tk.Frame(header, bg=PANEL)
heading.pack(side='left', padx=20, pady=10)
tk.Label(heading, text='QUADCOPTER', bg=PANEL, fg=TEXT,
         font=('DejaVu Sans', 16, 'bold')).pack(anchor='w')
tk.Label(heading, text='GROUND STATION  /  FLIGHT MONITOR', bg=PANEL, fg=MUTED,
         font=('DejaVu Sans', 8, 'bold')).pack(anchor='w')

status = tk.Frame(header, bg=PANEL)
status.pack(side='right', padx=20)
controller_label = tk.Label(status, text='● CONTROLLER: CHECKING', bg=PANEL, fg=MUTED,
                            font=('DejaVu Sans', 9, 'bold'))
controller_label.pack(side='right', padx=(16, 0))
telemetry_label = tk.Label(status, text='● TELEMETRY: WAITING', bg=PANEL, fg=MUTED,
                           font=('DejaVu Sans', 9, 'bold'))
telemetry_label.pack(side='right', padx=(16, 0))
rate_label = tk.Label(status, text='DISPLAY: -- Hz', bg=PANEL, fg=MUTED,
                      font=('DejaVu Sans', 9))
rate_label.pack(side='right')

controls_frame = tk.Frame(root, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
controls_frame.grid(row=1, column=0, sticky='nsew', padx=(12, 6), pady=12)
orientation_frame = tk.Frame(root, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
orientation_frame.grid(row=1, column=1, sticky='nsew', padx=(6, 12), pady=12)
log_frame = tk.Frame(root, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
log_frame.grid(row=2, column=0, columnspan=2, sticky='nsew', padx=12, pady=(0, 12))

logs = Logs(log_frame)
throttles = Throttles(controls_frame, onHeightThrottleChange, onYawThrottleChange,
                      onPitchThrottleChange, onRollThrottleChange)
telemetry = DroneTelemetry(orientation_frame)

# Controller library callbacks may run on its own thread. Queue only; never touch Tk here.
def controller_event(axis, value):
    try:
        value = float(value)
        if math.isfinite(value):
            enqueue_latest(controller_queue, (axis, value))
    except (TypeError, ValueError):
        pass


controller = None
try:
    controller = Controller(
        lambda v: controller_event('yaw', v),
        lambda v: controller_event('height', v),
        lambda v: controller_event('roll', v),
        lambda v: controller_event('pitch', v),
    )
    connected = getattr(controller, 'controller', None) is not None
    controller_label.configure(text='● CONTROLLER: ACTIVE' if connected else '● CONTROLLER: NOT FOUND',
                               fg=GREEN if connected else ORANGE)
except Exception as exc:
    controller_label.configure(text='● CONTROLLER: ERROR', fg=ORANGE)
    logs.update_log(f'WARNING Controller initialization failed: {exc}')


def log_callback(encoded_message):
    try:
        enqueue_latest(logs_queue, encoded_message.decode(errors='replace'))
    except Exception:
        pass


def telemetry_callback(json_message):
    try:
        message = json.loads(json_message.decode())
        roll, pitch = float(message['roll']), float(message['pitch'])
        if math.isfinite(roll) and math.isfinite(pitch):
            enqueue_latest(telemetry_queue, (roll, pitch, time.monotonic()))
    except (UnicodeError, ValueError, TypeError, KeyError):
        pass


last_packet = None
last_display = None
closing = False


def process_queues():
    global last_packet, last_display
    if closing:
        return
    for _ in range(50):
        try:
            logs.update_log(logs_queue.get_nowait())
        except queue.Empty:
            break

    # Apply only the newest value for each stick axis, on the GUI thread.
    latest_controls = {}
    for _ in range(200):
        try:
            axis, value = controller_queue.get_nowait()
            latest_controls[axis] = value
        except queue.Empty:
            break
    for axis, value in latest_controls.items():
        throttles.set_control(axis, value)

    latest = None
    for _ in range(100):
        try:
            latest = telemetry_queue.get_nowait()
        except queue.Empty:
            break
    now = time.monotonic()
    if latest is not None:
        roll, pitch, last_packet = latest
        telemetry.update(roll, pitch)
        if last_display is not None and now > last_display:
            rate_label.configure(text=f'DISPLAY: {1 / (now - last_display):.0f} Hz')
        last_display = now
    if last_packet is None:
        telemetry_label.configure(text='● TELEMETRY: WAITING', fg=MUTED)
        rate_label.configure(text='DISPLAY: -- Hz')
    elif now - last_packet > 2:
        telemetry_label.configure(text='● TELEMETRY: STALE', fg=ORANGE)
        rate_label.configure(text='DISPLAY: -- Hz')
    else:
        telemetry_label.configure(text='● TELEMETRY: RECEIVING', fg=GREEN)
    root.after(50, process_queues)


# Retain references: subscriber lifecycle is managed by your existing ZeroMQ implementation.
logs_subscriber = ZeroMQSubscriber(host=DRONE_IP, topic='logs', callback=log_callback)
telemetry_subscriber = ZeroMQSubscriber(host=DRONE_IP, topic='telemetry', callback=telemetry_callback)


def on_close():
    global closing
    closing = True
    # Explicit shutdown where supported; do not depend on Controller.__del__.
    if controller is not None:
        device = getattr(controller, 'controller', None)
        if device is not None:
            try:
                device.deactivate()
            except Exception:
                pass
    root.destroy()


root.protocol('WM_DELETE_WINDOW', on_close)
root.after(50, process_queues)
root.mainloop()
