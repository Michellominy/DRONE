import threading
from zeroMQManager import ZeroMQManager

class ZeroMQLogger:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialize()
        return cls._instance

    def _initialize(self):
        self.publisher = ZeroMQManager(role="server", mode="PUBSUB")

    def log(self, level, message):
        log_entry = {"level": level, "message": message}
        self.publisher.send("log", log_entry)

    def info(self, message):
        self.log("INFO", message)

    def warning(self, message):
        self.log("WARNING", message)

    def error(self, message):
        self.log("ERROR", message)
