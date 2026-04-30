#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class WallIdPublisher(Node):
    def __init__(self):
        super().__init__('wall_id_publisher')
        self.declare_parameter('wall_id_red',  23)
        self.declare_parameter('wall_id_blue',  0)

        red_id  = self.get_parameter('wall_id_red').value
        blue_id = self.get_parameter('wall_id_blue').value

        pub = self.create_publisher(Int32MultiArray, '/arena/wall_ids', 10)
        msg = Int32MultiArray()
        msg.data = [red_id, blue_id]

        def _pub():
            pub.publish(msg)
        self.create_timer(1.0, _pub)
        self.get_logger().info(f'Publishing wall IDs: red={red_id} blue={blue_id}')

def main():
    rclpy.init()
    node = WallIdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()