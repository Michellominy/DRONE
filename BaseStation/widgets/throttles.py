import tkinter as tk


class Throttles:
    def __init__(self, root, onHeightThrottleChange, onYawThrottleChange, onPitchThrottleChange, onRollThrottleChange):
        self.height_throttle = tk.IntVar()
        self.yaw_throttle = tk.IntVar()
        self.pitch_throttle = tk.IntVar()
        self.roll_throttle = tk.IntVar()
                        
        self.height_slider = tk.Scale(root, from_=100, to=-100, orient=tk.VERTICAL, length=200, variable=self.height_throttle, command=self.on_height_throttle_change)
        self.height_slider.grid(column=0, row=0, columnspan=1, rowspan=2)
        self.yaw_slider = tk.Scale(root, from_=-100, to=100, orient=tk.HORIZONTAL, length=200, variable=self.yaw_throttle, command=self.on_yaw_throttle_change)
        self.yaw_slider.grid(column=0, row=2, columnspan=1, rowspan=1)

        self.pitch_slider = tk.Scale(root, from_=100, to=-100, orient=tk.VERTICAL, length=200, variable=self.pitch_throttle, command=self.on_pitch_throttle_change)
        self.pitch_slider.grid(column=1, row=0, columnspan=1, rowspan=2)
        self.roll_slider = tk.Scale(root, from_=-100, to=100, orient=tk.HORIZONTAL, length=200, variable=self.roll_throttle, command=self.on_roll_throttle_change)
        self.roll_slider.grid(column=1, row=2, columnspan=1, rowspan=1)

        self.onHeightThrottleChange = onHeightThrottleChange
        self.onYawThrottleChange = onYawThrottleChange
        self.onPitchThrottleChange = onPitchThrottleChange
        self.onRollThrottleChange = onRollThrottleChange
    
    def on_height_throttle_change(self, val):
        print(f"Height Throttle changed to: {val}")
        self.onHeightThrottleChange(val)
        
    def on_yaw_throttle_change(self, val):
        print(f"Yaw Throttle changed to: {val}")
        self.onYawThrottleChange(val)
    
    def on_pitch_throttle_change(self, val):
        print(f"Pitch Throttle changed to: {val}")
        self.onPitchThrottleChange(val)
        
    def on_roll_throttle_change(self, val):
        print(f"Roll Throttle changed to: {val}")
        self.onRollThrottleChange(val)
