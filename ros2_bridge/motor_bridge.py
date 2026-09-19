import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        self.state_pub = self.create_publisher(Float32, 'motor_state', 10)
        self.cmd_sub = self.create_subscription(Float32, 'cmd_vel', self.cmd_callback, 10)

        self.timer = self.create_timer(0.02, self.read_serial)
        self.get_logger().info('Motor bridge started')

    def cmd_callback(self, msg):
        command = f"{msg.data}\n"
        self.ser.write(command.encode())
        self.get_logger().info(f'Sent setpoint: {msg.data}')

    def read_serial(self):
        try:
            line = self.ser.readline().decode().strip()
            if line:
                parts = line.split(',')
                if len(parts) == 3:              # time,setpoint,speed
                    speed = float(parts[2])
                    msg = Float32()
                    msg.data = speed
                    self.state_pub.publish(msg)
        except (ValueError, UnicodeDecodeError):
            pass

def main():
    rclpy.init()
    node = MotorBridge()
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