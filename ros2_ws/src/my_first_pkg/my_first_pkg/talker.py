import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.i = 0
    def timer_cb(self):
        msg = String()
        msg.data = f'hello {self.i}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
