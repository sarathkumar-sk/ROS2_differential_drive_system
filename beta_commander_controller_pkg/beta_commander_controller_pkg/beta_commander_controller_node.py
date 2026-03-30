import rclpy
from rclpy.node import Node
import math
import time

from sfr_coursework1_interface_package.msg import WheelAngularVelocities, TaskSpacePose
from sfr_coursework1_interface_package.srv import TurnRobotOn, TurnRobotOff


class ControllerNode(Node):
    def __init__(self):
        super().__init__('beta_commander_controller_node')
        desired_angle_deg = 4.0
        self.desired_angle_rad = math.radians(desired_angle_deg)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_phi = 0.0

        self.wheel_pub = self.create_publisher(
            msg_type=WheelAngularVelocities,
            topic='/beta_commander/wheel_angular_velocities',
            qos_profile=1
        )

        self.create_subscription(
            msg_type=TaskSpacePose,
            topic='/beta_commander/task_space_pose',
            callback=self.pose_callback,
            qos_profile=1
        )

        self.turn_on_client = self.create_client(srv_type=TurnRobotOn, srv_name='beta_commander/turn_robot_on')
        self.turn_off_client = self.create_client(srv_type=TurnRobotOff, srv_name='beta_commander/turn_robot_off')

        self.get_logger().info("Waiting for turn_robot_on service...")
        self.turn_on_client.wait_for_service()

        self.get_logger().info("Turning beta commander ON.")
        self.turn_on_client.call_async(TurnRobotOn.Request())

        time.sleep(1.0)

        self.rotate_to_angle()
        self.move_forward()

        self.publish_wheel_speeds(0.0, 0.0)
        time.sleep(0.5)

        self.turn_off_client.call_async(TurnRobotOff.Request())

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_phi = msg.phi_z

    def publish_wheel_speeds(self, w_r, w_l):
        msg = WheelAngularVelocities()
        msg.right_wheel_angular_velocity = float(w_r)
        msg.left_wheel_angular_velocity = float(w_l)
        self.wheel_pub.publish(msg)

    def rotate_to_angle(self):
        self.get_logger().info("Rotating Beta commander")
        start = time.time()
        Kp = 2.0  
        tolerance = 0.001 
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            error = self.desired_angle_rad - self.current_phi
            if abs(error) < tolerance:
                break
            w = Kp * error
            w = max(-0.1, min(0.1, w))
            self.publish_wheel_speeds(w, -w)
            if time.time() - start > 60.0:
                final_angle_deg = math.degrees(self.current_phi)
                final_angle_deg = round(final_angle_deg)
                self.get_logger().info(f"Rotation timeout reached. Final angle: {final_angle_deg:.2f}")
                break
        self.publish_wheel_speeds(0.0, 0.0)
        time.sleep(0.5)
        final_angle_deg = math.degrees(self.current_phi)
        final_angle_deg = round(final_angle_deg)
        self.get_logger().info(f"Rotation complete. Final angle: {final_angle_deg:.2f}")

    def move_forward(self, distance=1.0):
        self.get_logger().info("Moving forward 1 meter")

        start_x = self.current_x
        start_y = self.current_y
        start = time.time()

        w = 5.0
        self.publish_wheel_speeds(w, w)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            dx = self.current_x - start_x
            dy = self.current_y - start_y
            travelled = math.sqrt(dx*dx + dy*dy)

            if travelled >= distance:
                break

            if time.time() - start > 60.0:
                travelled = round(travelled)
                self.get_logger().info(f"Forward movement timeout reached. Total distance travelled: {travelled:.2f} m")
                break

        self.publish_wheel_speeds(0.0, 0.0)
        time.sleep(0.5)
        travelled = round(travelled)
        self.get_logger().info(f"Forward motion complete. Total distance travelled: {travelled:.2f} m")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()
