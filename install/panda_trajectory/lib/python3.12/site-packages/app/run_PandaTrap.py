import rclpy
import panda_py
import os
from panda_trajectory.PandaTrap import PandaTrapNode 

def main(args=None):
    rclpy.init(args=args)
    
    robot_ip = "192.168.1.11"
    desk = panda_py.Desk(robot_ip, "BRL", "IITADVRBRL")
    panda = panda_py.Panda(robot_ip)
   
    desk.unlock()
    desk.activate_fci()

    try:
        PandaTrapNode(panda)
    except KeyboardInterrupt:
        print("Stopping robot...")
    finally:
        desk.deactivate_fci()
        desk.lock()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
