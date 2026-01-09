import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import time

# ICM-20948 Registers
BANK_SEL = 0x7F
I2C_ADDR = 0x68

# Bank 0 Registers
WHO_AM_I = 0x00
USER_CTRL = 0x03
PWR_MGMT_1 = 0x06
ACCEL_XOUT_H = 0x2D
GYRO_XOUT_H = 0x33

class Icm20948Publisher(Node):
    def __init__(self):
        super().__init__('icm20948_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        self.bus = smbus.SMBus(1)

        self.init_sensor()
        self.timer = self.create_timer(0.005, self.timer_callback) # 200Hz
        self.get_logger().info('ICM-20948 Publisher Started at 200Hz')

    def select_bank(self, bank):
        self.bus.write_byte_data(I2C_ADDR, BANK_SEL, bank << 4)

    def init_sensor(self):
        # 1. Select Bank 0
        self.select_bank(0)
        # 2. Wake up: Clear SLEEP bit (bit 6) and set clock to Auto (bits 2:0)
        self.bus.write_byte_data(I2C_ADDR, PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # 3. Disable I2C Master mode to prevent interference (optional but safer)
        self.bus.write_byte_data(I2C_ADDR, USER_CTRL, 0x00)

    def read_word(self, reg):
        high = self.bus.read_byte_data(I2C_ADDR, reg)
        low = self.bus.read_byte_data(I2C_ADDR, reg + 1)
        val = (high << 8) + low
        return val - 65536 if val > 32767 else val

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Default Scales: Accel +/- 2g (16384 LSB/g), Gyro +/- 250 dps (131 LSB/dps)
        G = 9.80665
        DEG_TO_RAD = 3.14159 / 180.0

        try:
            self.select_bank(0)
            # Accelerometer
            msg.linear_acceleration.x = (self.read_word(ACCEL_XOUT_H) / 16384.0) * G
            msg.linear_acceleration.y = (self.read_word(ACCEL_XOUT_H + 2) / 16384.0) * G
            msg.linear_acceleration.z = (self.read_word(ACCEL_XOUT_H + 4) / 16384.0) * G

            # Gyroscope
            msg.angular_velocity.x = (self.read_word(GYRO_XOUT_H) / 131.0) * DEG_TO_RAD
            msg.angular_velocity.y = (self.read_word(GYRO_XOUT_H + 2) / 131.0) * DEG_TO_RAD
            msg.angular_velocity.z = (self.read_word(GYRO_XOUT_H + 4) / 131.0) * DEG_TO_RAD

            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"I2C Read Error: {e}")

def main():
    rclpy.init()
    node = Icm20948Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
