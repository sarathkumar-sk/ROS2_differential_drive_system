import rclpy
from rclpy.node import Node
from sfr_coursework1_interface_package.srv import TurnRobotOn, TurnRobotOff
from sfr_coursework1_interface_package.msg import WheelAngularVelocities, TaskSpacePose
import math

class BetaCommanderNode(Node):

    def __init__(self):
        super().__init__('beta_commander_node')

        self.beta_commander_on = False #False: Off-state True: On-state
        
        self.x = 0.0
        self.y = 0.0
        self.phi_z = 0.0

        self.r = 0.05   # wheel radius [m]
        self.l = 0.1    # wheelbase [m]
        self.v_min = -0.1  # lower clamp [m/s]
        self.v_max = 0.1   # upper clamp [m/s]

        self.w_r = 0.0  # right wheel angular velocity (rad/s)
        self.w_l = 0.0  # left wheel angular velocity (rad/s)


        self.turn_on_service_server = self.create_service(
            srv_type= TurnRobotOn, 
            srv_name='beta_commander/turn_robot_on',
            callback=self.turn_on_callback)  
          
        self.turn_off_service_server = self.create_service(
            srv_type= TurnRobotOff, 
            srv_name='beta_commander/turn_robot_off',
            callback=self.turn_off_callback)
        
        self.beta_publisher = self.create_publisher(msg_type=TaskSpacePose, topic='/beta_commander/task_space_pose', qos_profile=1)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publisher_callback)

        self.beta_subscriber = self.create_subscription(msg_type=WheelAngularVelocities, topic='/beta_commander/wheel_angular_velocities',callback=self.subscriber_callback,qos_profile=1)

    def turn_on_callback(self,request: TurnRobotOn.Request,response: TurnRobotOn.Response) -> TurnRobotOn.Response:
        if not self.beta_commander_on:
            self.beta_commander_on = True
            response.success = True
            self.get_logger().info("Beta commander turned ON")
        else:
            response.success = False
            self.get_logger().info("Turn ON requested but beta commander already ON")
        return response
    
    def turn_off_callback(self,request: TurnRobotOff.Request,response: TurnRobotOff.Response) -> TurnRobotOff.Response:
        if self.beta_commander_on:
            self.beta_commander_on = False
            self.w_r = 0.0
            self.w_l = 0.0
            response.success = True
            self.get_logger().info("Beta commander turned OFF")
        else:
            response.success = False
            self.get_logger().info("Turn OFF requested but Beta commander already OFF")
        return response
    
    def publisher_callback(self):
        if not self.beta_commander_on:
            return

        v_r = self.r * self.w_r
        v_l = self.r * self.w_l
        v = (v_r + v_l) / 2.0
        w = (v_r - v_l) / self.l

        self.x += v * math.cos(self.phi_z) * self.timer_period
        self.y += v * math.sin(self.phi_z) * self.timer_period
        self.phi_z += w * self.timer_period
        self.phi_z = (self.phi_z + math.pi) % (2 * math.pi) - math.pi

        msg = TaskSpacePose()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.phi_z = float(self.phi_z)
        self.beta_publisher.publish(msg)
    
    def subscriber_callback(self, msg):
        if not self.beta_commander_on:
            return 

        v_r = self.r * msg.right_wheel_angular_velocity
        v_l = self.r * msg.left_wheel_angular_velocity

        v_r = max(self.v_min, min(self.v_max, v_r))
        v_l = max(self.v_min, min(self.v_max, v_l))

        self.w_r = v_r / self.r
        self.w_l = v_l / self.r

def main(args=None):
    try:
        rclpy.init(args=args)
        beta_commander_node = BetaCommanderNode()
        rclpy.spin(beta_commander_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
