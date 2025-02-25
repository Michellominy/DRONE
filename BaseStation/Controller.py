from dualsense_controller import DualSenseController


class Controller:
    def __init__(self, onLeftStickChangeX, onLeftStickChangeY, onRightStickChangeX, onRightStickChangeY):
        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            raise Exception('No DualSense Controller available.')

        self.controller = DualSenseController()

        self.controller.left_stick_x.on_change(onLeftStickChangeX)
        self.controller.left_stick_y.on_change(onLeftStickChangeY)
        self.controller.right_stick_x.on_change(onRightStickChangeX)
        self.controller.right_stick_y.on_change(onRightStickChangeY)
        
        self.controller.activate()

    def __del__(self):
        self.controller.deactivate()
