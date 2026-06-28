from dualsense_controller import DualSenseController


class Controller:
    def __init__(self, onLeftStickChangeX, onLeftStickChangeY, onRightStickChangeX, onRightStickChangeY):
        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            # raise Exception('No DualSense Controller available.')
            print('Warning: No DualSense Controller available.')
            return
            

        self.controller = DualSenseController()

        self.controller.left_stick_x.on_change(lambda val : onLeftStickChangeX(val*100))
        self.controller.left_stick_y.on_change(lambda val : onLeftStickChangeY(val*100))
        self.controller.right_stick_x.on_change(lambda val : onRightStickChangeX(val*100))
        self.controller.right_stick_y.on_change(lambda val : onRightStickChangeY(val*100))
        
        self.controller.activate()

    def __del__(self):
        self.controller.deactivate()
