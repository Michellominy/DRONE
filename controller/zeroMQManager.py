import zmq
import threading
import json

class ZeroMQManager:
    def __init__(self, role, mode="PUBSUB"):
        self.context = zmq.Context()
        self.role = role
        self.mode = mode
        host = "0.0.0.0" # CHANGE ADDRESS IF NOT SERVER
        port = "5001"
        self.address = "tcp://{}:{}".format(host, port)
        
        if mode == "PUBSUB":
            if role == "server":
                self.socket = self.context.socket(zmq.PUB)
                self.socket.bind(self.address)
            else:
                self.socket = self.context.socket(zmq.SUB)
                self.socket.connect(self.address)
                self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        elif mode == "REQREP":
            if role == "server":
                self.socket = self.context.socket(zmq.REP)
                self.socket.bind(self.address)
            else:
                self.socket = self.context.socket(zmq.REQ)
                self.socket.connect(self.address)
    
    def send(self, message, topic):
        to_send = f"{topic} {json.dumps(message)}"
        if self.mode == "REQREP":
            self.socket.send(to_send)
            return self.socket.recv_json()
        else:
            self.socket.send(to_send)
    
    def receive(self, callback):
        def listen():
            while True:
                message = self.socket.recv_json()
                callback(message)
        
        thread = threading.Thread(target=listen, daemon=True)
        thread.start()
    
    def close(self):
        self.socket.close()
        self.context.term()
