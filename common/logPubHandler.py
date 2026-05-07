from zmq.log.handlers import PUBHandler

class LogPUBHandler(PUBHandler):
    def __init__(self, socket):
        super().__init__(socket)
        self.socket = socket

    def emit(self, record):
        try:
            msg = self.format(record).encode()
            self.socket.send_multipart([b"logs", msg])  
        except Exception:
            self.handleError(record)