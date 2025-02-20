import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from ahrs import Quaternion as AHRSQuaternion, DCM, RAD2DEG
import numpy as np
import imufusion
import serial
import json
import time
import threading

class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.ahrs = imufusion.Ahrs()
        self.offset = imufusion.Offset(100)
        self.time_now = time.time()
        self.is_data_ready = False
        self.gy_offset = np.zeros(3)
        self.hi_offset = np.zeros((3))
        self.time_diff = 0
        self.gy_calibrated = False
        self.data = None  # Initialize self.data

        self.collect_serial_data_thread = threading.Thread(target=self.collect_serial_data, args=("/dev/ttyUSB0",))
        self.collect_serial_data_thread.start()

        while not self.is_data_ready:
            self.get_logger().info('--- Veri Hazır Olana Kadar Bekleniyor ---')
            time.sleep(0.1)

        self.calibrate_gyro()
        # self.calibrate_mag()

        self.timer = self.create_timer(0.01, self.publish_imu_data)  # Zamanlayıcı hızını ihtiyacınıza göre ayarlayın

    def collect_serial_data(self, port):
        with serial.Serial(port, 115200) as ser:
            while True:
                if not ser.is_open:
                    ser.open()
                try:
                    line = ser.readline().decode('utf-8').strip()
                    print(f"Ham Seri Verisi: {line}") # Ham veriyi yazdır
                    data = json.loads(line)
                    print(f"JSON Verisi: {data}") # Ayrıştırılmış veriyi yazdır
                    if self.gy_calibrated:
                        data["GYR"] = self.offset.update(np.array(data["GYR"]) - self.gy_offset)
                        self.data = data
                        self.ahrs.update_no_magnetometer(np.array(self.data["GYR"]),
                                            np.array(self.data["ACC"]),
                                            #np.array(self.data["MAG"]) - self.hi_offset,
                                            time.time() - self.time_now)
                        self.time_now = time.time()
                        self.w, self.x, self.y, self.z = self.ahrs.quaternion.wxyz
                        print(f"Quaternion: w={self.w}, x={self.x}, y={self.y}, z={self.z}") # Quaternion değerlerini yazdır
                        self.Q = AHRSQuaternion([self.w, self.x, self.y, self.z])
                        self.roll, self.pitch, self.yaw = DCM(self.Q.to_DCM()).to_rpy() * RAD2DEG
                        self.is_data_ready = True
                    else:
                        self.data = data
                        self.is_data_ready = True
                except json.JSONDecodeError:
                    self.get_logger().error("JSON parse hatası: Geçersiz veri formatı")
                except KeyError as e:
                    self.get_logger().error(f"JSON anahtar hatası: {e}")
                except ValueError as e:
                    self.get_logger().error(f"JSON verisi float türüne çevrilemedi: {e}")
                except Exception as e:
                    self.get_logger().error(f"IMU Verisi Okuma Hatası: {e}")

    def get_rpy(self):
        return self.roll, self.pitch, self.yaw

    def calibrate_gyro(self):
        gy_x, gy_y, gy_z = 0, 0, 0
        counter = 0
        while counter < 1000:
            gy_x += self.data["GYR"][0]
            gy_y += self.data["GYR"][1]
            gy_z += self.data["GYR"][2]
            counter += 1
            self.get_logger().info(f"Gyro kalibrasyonu, sabit tutun % {counter/10}")
            time.sleep(0.005)
        gy_x /= 1000
        gy_y /= 1000
        gy_z /= 1000
        self.gy_offset[0] = gy_x
        self.gy_offset[1] = gy_y
        self.gy_offset[2] = gy_z
        self.gy_calibrated = True

    def publish_imu_data(self):
        if self.is_data_ready and self.data is not None:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Ensure the orientation is of type Quaternion
            imu_msg.orientation.x = float(self.x)
            imu_msg.orientation.y = float(self.y)
            imu_msg.orientation.z = float(self.z)
            imu_msg.orientation.w = float(self.w)

            imu_msg.angular_velocity.x = float(self.data["GYR"][0])
            imu_msg.angular_velocity.y = float(self.data["GYR"][1])
            imu_msg.angular_velocity.z = float(self.data["GYR"][2])

            imu_msg.linear_acceleration.x = float(self.data["ACC"][0])
            imu_msg.linear_acceleration.y = float(self.data["ACC"][1])
            imu_msg.linear_acceleration.z = float(self.data["ACC"][2])

            self.publisher_.publish(imu_msg)
            self.get_logger().info(f"IMU Yayınlandı | Roll: {self.roll:.2f}, Pitch: {self.pitch:.2f}, Yaw: {self.yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()