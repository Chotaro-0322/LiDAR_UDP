import rclpy
from rclpy.node import Node
from .lidar_udp_catch import Lidar_udp_catch

def main(args=None):
    rclpy.init(args=args)
    listener = Lidar_udp_catch()
    while True:
        rclpy.spin_once(listener, timeout_sec=2)

if __name__ == "__main__":
    main()