import zmq
import threading
import json
import logging
from common.logPubHandler import LogPUBHandler

            
class ZeroMQSubscriber:
    def __init__(self, host, topic, callback):
        self.context = zmq.Context()
        self.host = host 
        port = "5001"
        self.address = "tcp://{}:{}".format(self.host, port)
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.receive(callback)
        
    def receive(self, callback):
        def listen():
            while True:
                topic, message = self.socket.recv_multipart()
                callback(message)
        
        thread = threading.Thread(target=listen, daemon=True)
        thread.start()
        
class ZeroMQServer:
    def __init__(self):
        self.context = zmq.Context()
        host = "0.0.0.0"
        port = "5001"
        self.address = "tcp://{}:{}".format(host, port)
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(self.address)

    def send(self, topic, message):
        self.socket.send_multipart([f"{topic}".encode('UTF-8'), json.dumps(message).encode()])
            
    def setLogger(self):
        zmq_log_handler = LogPUBHandler(self.socket)
        logger = logging.getLogger()
        logger.addHandler(zmq_log_handler)
    
    def close(self):
        self.socket.close()
        self.context.term()

zeroMQServer = ZeroMQServer()

"""
TOPICS:
- logs: for log messages
    * DRONE => BASESTATION
    * data: string log message
- gyro: for receiving gyro data
    * DRONE => BASESTATION
    * data: {yaw: float, pitch: float, roll: float}
- accel: for receiving accelerometer data
    * DRONE => BASESTATION
    * data: {x: float, y: float, z: float}
- throttle: for sending throttle commands
    * BASESTATION => DRONE
    * data: int motor throttle value
- yaw: for sending yaw commands
    * BASESTATION => DRONE
    * data: int yaw value
- pitch: for sending pitch commands
    * BASESTATION => DRONE
    * data: int pitch value
- roll: for sending roll commands
    * BASESTATION => DRONE
    * data: int roll value
"""