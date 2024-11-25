from pynq.lib import MicroblazeLibrary
from pynq.overlays.base import BaseOverlay

from constant import PMOD_SDA_PIN, PMOD_SCL_PIN

base = BaseOverlay("base.bit")
lib = MicroblazeLibrary(base.PMODB, ['i2c'])

class pynq_i2c:
    def __init__(self):
        self.i2c_device = lib.i2c_open(PMOD_SDA_PIN, PMOD_SCL_PIN)
    
    def iic_writeByte(self, devAddr, regAddr, data):
        temp = bytearray(2)
        temp[0] = regAddr
        temp[1] = data
        self.i2c_device.write(devAddr, temp, 2)
    
    
    def iic_readByte(self, devAddr, regAddr):
        temp = bytearray(1)
        temp[0] = regAddr
        self.i2c_device.write(devAddr, temp, 1)
        self.i2c_device.read(devAddr, temp, 1)    
        return temp[0]
    
    def close(self):
        self.i2c_device.close()
        
pynq_i2c_instance = pynq_i2c()
