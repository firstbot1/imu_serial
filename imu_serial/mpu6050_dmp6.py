import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_quaternion_node')
        self.pub = self.create_publisher(QuaternionStamped, 'imu_quaternion', 10)
        
        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        self.ser.flush()

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.callback)  # 100Hz

    def callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').rstrip()
            data = line.split(",")
            
            # 데이터의 정합성 체크
            if len(data) != 5 or not data[0] == "quat":
                self.ser.flushInput()  # 버퍼를 비워줍니다.
                return
            
            try:
                quat_stamped = QuaternionStamped()
                quat_stamped.header.stamp = self.get_clock().now().to_msg()
                quat_stamped.header.frame_id = "imu_frame"
                quat_stamped.quaternion.w = float(data[1])
                quat_stamped.quaternion.x = float(data[2])
                quat_stamped.quaternion.y = float(data[3])
                quat_stamped.quaternion.z = float(data[4])
                    
                self.pub.publish(quat_stamped)

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "base_link"
                t.child_frame_id = "imu_frame"
                t.transform.rotation = quat_stamped.quaternion
                self.br.sendTransform(t)

            except Exception as e:
                self.get_logger().info("Error parsing data: " + str(e))

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


