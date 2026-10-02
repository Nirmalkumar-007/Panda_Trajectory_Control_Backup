import rclpy
import panda_py

from panda_trajectory.PandaTrajectory import PandaTrajectoryNode


def main(args=None):
    rclpy.init(args=args)

    desk = panda_py.Desk("192.168.1.11", "BRL", "IITADVRBRL")
    panda = panda_py.Panda("192.168.1.11")

    desk.unlock()
    desk.activate_fci()

    try:
        PandaTrajectoryNode(desk, panda)
    except KeyboardInterrupt:
        print("Stopping robot...")

    desk.deactivate_fci()
    desk.lock()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
