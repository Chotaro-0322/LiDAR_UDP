import rclpy
from rclpy.node import Node
from .lidar_udp_catch import Lidar_udp_catch

def main(args=None):
    rclpy.init(args=args)
    listener = Lidar_udp_catch()
    listener.prepare_anime()
    listener.run()
    while rclpy.ok():
        rclpy.spin_once(listener, timeout_sec=0)
        # listener.run()
        # print("hello!!!")

if __name__ == "__main__":
    main()