import tk_tools


class DroneOrientation:
    def __init__(self, root):                
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
