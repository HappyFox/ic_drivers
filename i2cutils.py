class I2cRegDevice:

    def __init__(self, i2c, address):
        self.i2c = i2c
        self.address = address

    def __getitem__(self, key):
        return self.i2c.readfrom_mem(self.address, key, 1)[0]

    def __setitem__(self, key, value):
        self.i2c.writeto_mem(self.address, key, bytearray([value]))

    def readfrom_mem(self, mem_addr, nbytes):
        return self.i2c.readfrom_mem(self.address, mem_addr, nbytes)

    def readfrom_mem_into(self, mem_addr, buf):
        return self.i2c.readfrom_mem_into(self.address, mem_addr, buf)

    def writeto_mem(self, mem_addr, buf):
        return self.i2c.writeto_mem(self.address, mem_addr, buf)
