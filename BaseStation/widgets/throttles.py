import tkinter as tk


class Throttles:
    def __init__(self, root, onHeightThrottleChange):
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
    
    def update_height_throttle(self, new_val):
        self.height_throttle.set(new_val * 100)
        self.onHeightThrottleChange(new_val)
        print(f"udpated height: {self.height_throttle}, new_val: {new_val}")
        
    def update_yaw_throttle(self, new_val):
        self.yaw_throttle.set(new_val * 100)
        print(f"udpated yaw: {self.yaw_throttle}, new_val: {new_val}")
    
    def update_pitch_throttle(self, new_val):
        self.pitch_throttle.set(new_val * 100)
        
    def update_roll_throttle(self, new_val):
        self.roll_throttle.set(new_val * 100)

    
    def on_height_throttle_change(self, val):
        print(f"Height Throttle changed to: {val}")
        
    def on_yaw_throttle_change(self, val):
        print(f"Yaw Throttle changed to: {val}")
    
    def on_pitch_throttle_change(self, val):
        print(f"Pitch Throttle changed to: {val}")
        
    def on_roll_throttle_change(self, val):
        print(f"Roll Throttle changed to: {val}")
