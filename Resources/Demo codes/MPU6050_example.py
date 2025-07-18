import time
from machine import I2C, Pin

class MPU6050:
    """
    MicroPython class for MPU6050 accelerometer and gyroscope sensor
    """
    
    # MPU6050 I2C address
    MPU6050_ADDR = 0x68
    
    # Register addresses
    PWR_MGMT_1 = 0x6B      # Power Management register
    GYRO_CONFIG = 0x1B     # Gyroscope Configuration register
    ACCEL_CONFIG = 0x1C    # Accelerometer Configuration register
    ACCEL_OUT = 0x3B    # Accelerometer Data register
    GYRO_OUT = 0x43     # Gyroscope Data register
    
    def __init__(self, i2c, addr=MPU6050_ADDR):
        """
        Initialize MPU6050 sensor
        
        Args:
            i2c: I2C object (machine.I2C)
            addr: I2C address of MPU6050 (default: 0x68)
        """
        self.i2c = i2c
        self.addr = addr
        
        # Initialize sensor variables
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        
        self.g_force_x = 0.0
        self.g_force_y = 0.0
        self.g_force_z = 0.0
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0
        
        # Setup the sensor
        self.setup_mpu()
        
    def setup_mpu(self):
        """
        Configure MPU6050 registers - equivalent to setupMPU() in Arduino code
        """
        # Power Management 1 - wake up the sensor
        self.i2c.writeto_mem(self.addr, self.PWR_MGMT_1, bytes([0x00]))
        
        # Gyroscope Configuration - set to ±250°/s
        self.i2c.writeto_mem(self.addr, self.GYRO_CONFIG, bytes([0x00]))
        
        # Accelerometer Configuration - set to ±2g
        # If set to ±16g, please writeto_mem 0b00011000
        self.i2c.writeto_mem(self.addr, self.ACCEL_CONFIG, bytes([0x00]))
        
        # print("MPU6050 initialized successfully!")
        
    def read_accel_registers(self):
        """
        Read accelerometer data from registers 0x3B to 0x40
        """
        # Read 6 bytes starting from ACCEL_OUT
        data = self.i2c.readfrom_mem(self.addr, self.ACCEL_OUT, 6)
        
        # Convert bytes to signed 16-bit integers
        self.accel_x = self._bytes_to_int(data[0], data[1])
        self.accel_y = self._bytes_to_int(data[2], data[3])
        self.accel_z = self._bytes_to_int(data[4], data[5])
        
        # Convert to g-force (multiply by 100 like in Arduino code)
        self.g_force_x = self.accel_x / 16384.0 * 100
        self.g_force_y = self.accel_y / 16384.0 * 100
        self.g_force_z = self.accel_z / 16384.0 * 100
        
    def read_gyro_registers(self):
        """
        Read gyroscope data from registers 0x43 to 0x48
        """
        # Read 6 bytes starting from GYRO_OUT
        data = self.i2c.readfrom_mem(self.addr, self.GYRO_OUT, 6)
        
        # Convert bytes to signed 16-bit integers
        self.gyro_x = self._bytes_to_int(data[0], data[1])
        self.gyro_y = self._bytes_to_int(data[2], data[3])
        self.gyro_z = self._bytes_to_int(data[4], data[5])
        
        # Convert to degrees per second
        self.rot_x = self.gyro_x / 131.0
        self.rot_y = self.gyro_y / 131.0
        self.rot_z = self.gyro_z / 131.0
        
    def _bytes_to_int(self, high_byte, low_byte):
        """
        Convert two bytes to signed 16-bit integer
        Equivalent to Wire.read()<<8|Wire.read() in Arduino
        """
        value = (high_byte << 8) | low_byte
        # Convert to signed integer
        if value > 32767:
            value -= 65536
        return value
        
    def read_all(self):
        """
        Read both accelerometer and gyroscope data
        """
        self.read_accel_registers()
        self.read_gyro_registers()
        
    def print_data(self):
        """
        Print sensor data in the same format as Arduino code
        """
        print(f"x-a  {self.rot_x}      x-g  {self.g_force_x}")
        print(f"y-a  {self.rot_y}      y-g  {self.g_force_y}")
        print(f"z-a  {self.rot_z}      z-g  {self.g_force_z}")
        print("")
        
    def get_accel_data(self):
        """
        Get accelerometer data as tuple (x, y, z) in g-force
        """
        return (self.g_force_x, self.g_force_y, self.g_force_z)
        
    def get_gyro_data(self):
        """
        Get gyroscope data as tuple (x, y, z) in degrees per second
        """
        return (self.rot_x, self.rot_y, self.rot_z)
        
    def get_all_data(self):
        """
        Get all sensor data as dictionary
        """
        return {
            'accel': {
                'x': self.g_force_x,
                'y': self.g_force_y,
                'z': self.g_force_z
            },
            'gyro': {
                'x': self.rot_x,
                'y': self.rot_y,
                'z': self.rot_z
            }
        }


# Example usage:
def main():
    """
    Main function demonstrating usage - equivalent to Arduino setup() and loop()
    """
    # Initialize I2C (adjust pins according to your board)
    # For ESP32: SCL=Pin(22), SDA=Pin(21)
    # For ESP8266: SCL=Pin(5), SDA=Pin(4)
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
    
    # Create MPU6050 instance
    mpu = MPU6050(i2c)
    
    # Wait for initialization
    time.sleep(1)
    print("init ok!")
    
    # Main loop
    while True:
        # Read all sensor data
        mpu.read_all()
        
        # Print data in Arduino format
        mpu.print_data()
        
        # Wait 100ms like in Arduino code
        time.sleep_ms(100)


if __name__ == "__main__":
    main()