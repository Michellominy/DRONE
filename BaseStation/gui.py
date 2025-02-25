import tkinter as tk
import json
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from BaseStation import Controller
from BaseStation.widgets.drone_orientation import DroneOrientation
from BaseStation.widgets.logs import Logs
from BaseStation.widgets.throttles import Throttles
from common.constants import DRONE_IP
from common.zeroMQManager import ZeroMQSubscriber
        
        
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

throttles = Throttles(throttle_frame)
logs = Logs(log_frame)
orientation = DroneOrientation(orientation_frame)
# controlller = Controller(throttles.update_yaw_throttle, throttles.update_height_throttle, throttles.update_roll_throttle, throttles.update_pitch_throttle)

def send_throttle_values():
    left_x = throttles.left_x_slider.get()
    left_y = throttles.left_y_slider.get()
    right_x = throttles.right_x_slider.get()
    right_y = throttles.right_y_slider.get()
    
    log_message = f"Sending Throttle - Left: ({left_x}, {left_y}) | Right: ({right_x}, {right_y})"
    logs.update_log(log_message)


def log_callback(encoded_message):
    message = encoded_message.decode()
    logs.update_log(message)

def gyro_callback(json_message):
    message = json.loads(json_message.decode())
    print(f"DATA: {message}")

logs_subscriber = ZeroMQSubscriber(host=DRONE_IP, topic="logs", callback=log_callback)
gyro_subscriber = ZeroMQSubscriber(host=DRONE_IP, topic="gyro", callback=gyro_callback)


root.mainloop()