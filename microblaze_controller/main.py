import time
import logging
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from common.zeroMQManager import zeroMQServer, ZeroMQSubscriber
from common.constants import BASE_STATION_IP
from microblaze_controller.drone import Drone

zeroMQServer.setLogger()
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

drone = Drone()

input_throttle_subscriber = ZeroMQSubscriber(host=BASE_STATION_IP, topic="throttle", callback=drone.update_input_throttle_callback)


if __name__ == "__main__":
    input("Press key to start ...")

    try:
        drone.start()
        print("Press Ctrl+C to stop.")
        while True:
            drone.read_debug()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping PID loop...")
        drone.stop()
        print("Stopped.")
