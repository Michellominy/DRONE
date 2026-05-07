import tkinter as tk
import json
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from BaseStation.Controller import Controller
from BaseStation.widgets.drone_orientation import DroneOrientation
from BaseStation.widgets.logs import Logs
from BaseStation.widgets.throttles import Throttles
from common.constants import DRONE_IP
from common.zeroMQManager import ZeroMQSubscriber, zeroMQServer


def onHeightThrottleChange(new_height):
    zeroMQServer.send("throttle", {"data": new_height})

def onYawThrottleChange(new_yaw):
    zeroMQServer.send("yaw", {"data": new_yaw})

def onPitchThrottleChange(new_pitch):
    zeroMQServer.send("pitch", {"data": new_pitch})

def onRollThrottleChange(new_roll):
    zeroMQServer.send("roll", {"data": new_roll})


root = tk.Tk()
root.title("Drone Controller")
root.geometry("1200x800")


throttle_frame = tk.Frame(root)
throttle_frame.grid(column=0, row=0, columnspan=1, rowspan=1)
log_frame = tk.Frame(root)
log_frame.grid(column=0, row=1, columnspan=2, rowspan=1, sticky="nsew")
orientation_frame = tk.Frame(root)
orientation_frame.grid(column=1, row=0, columnspan=1, rowspan=1)

root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=1)
root.rowconfigure(0, weight=3)
root.rowconfigure(1, weight=1)

logs = Logs(log_frame)
throttles = Throttles(throttle_frame, onHeightThrottleChange, onYawThrottleChange, onPitchThrottleChange, onRollThrottleChange)
orientation = DroneOrientation(orientation_frame)
controller = Controller(throttles.on_yaw_throttle_change, throttles.on_height_throttle_change, throttles.on_roll_throttle_change, throttles.on_pitch_throttle_change)

def log_callback(encoded_message):
    message = encoded_message.decode()
    logs.update_log(message)

def gyro_callback(json_message):
    message = json.loads(json_message.decode())
    print(f"GYRO DATA: {message}")
    
def accel_callback(json_message):
    message = json.loads(json_message.decode())
    print(f"ACCEL DATA: {message}")

logs_subscriber = ZeroMQSubscriber(host=DRONE_IP, topic="logs", callback=log_callback)
gyro_subscriber = ZeroMQSubscriber(host=DRONE_IP, topic="gyro", callback=gyro_callback)
accel_subscriber = ZeroMQSubscriber(host=DRONE_IP, topic="accel", callback=accel_callback)  

root.mainloop()