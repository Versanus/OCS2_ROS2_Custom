#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

class JointStateComparator(Node):
    def __init__(self):
        super().__init__('joint_state_comparator')
        
        # Joint Names
        self.joint_names = [
            'FL_HipX', 'FL_HipY', 'FL_Knee',
            'FR_HipX', 'FR_HipY', 'FR_Knee',
            'HL_HipX', 'HL_HipY', 'HL_Knee',
            'HR_HipX', 'HR_HipY', 'HR_Knee'
        ]
        
        self.error_threshold = 0.1  # 0.1 rad ≈ 5.73 derece
        
        self.joint_states_data = None
        self.htdw_joint_states_data = None
        
        # Subscriber'lar
        self.joint_states_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10
        )
        
        self.htdw_joint_states_sub = self.create_subscription(
            JointState,
            'htdw_joint_state',
            self.htdw_joint_states_callback,
            10
        )
        
        # ---- ERROR TOPIC PUBLISHER ----
        self.error_pub = self.create_publisher(String, 'joint_state_errors', 10)

        self.timer = self.create_timer(0.5, self.compare_joint_states)
        
        # self.get_logger().info('Joint State Comparator node started.')
    
    def joint_states_callback(self, msg:JointState):
        self.joint_states_data = msg
    
    def htdw_joint_states_callback(self, msg:JointState):
        self.htdw_joint_states_data = msg
    
    def compare_joint_states(self):
        if self.joint_states_data is None or self.htdw_joint_states_data is None:
            return
        
        joint_states_dict = {}
        htdw_joint_states_dict = {}
        
        for name, position in zip(self.joint_states_data.name, self.joint_states_data.position):
            joint_states_dict[name] = position
        
        for name, position in zip(self.htdw_joint_states_data.name, self.htdw_joint_states_data.position):
            htdw_joint_states_dict[name] = position
        
        errors_found = False
        error_details = []
        
        for joint_name in self.joint_names:
            if joint_name in joint_states_dict and joint_name in htdw_joint_states_dict:
                pos1 = joint_states_dict[joint_name]
                pos2 = htdw_joint_states_dict[joint_name]
                
                difference = abs(pos1 - pos2)
                
                if difference > self.error_threshold:
                    errors_found = True
                    diff_deg = math.degrees(difference)
                    
                    error_msg = (
                        f"{joint_name}: "
                        f"joint_states={pos1:.4f}rad ({math.degrees(pos1):.2f}°), "
                        f"htdw_joint_state={pos2:.4f}rad ({math.degrees(pos2):.2f}°), "
                        f"Fark={difference:.4f}rad ({diff_deg:.2f}°)"
                    )
                    error_details.append(error_msg)
        
        # ---- Hata mesajını topic olarak yayınla ----
        msg = String()
        
        if errors_found:
            msg.data = "\n".join(error_details)
            # self.get_logger().warn(f"{len(error_details)} joint'te hata var, /joint_state_errors yayınlandı.")
        else:
            msg.data = "No error"
        
        self.error_pub.publish(msg)
    
    def set_threshold(self, threshold):
        self.error_threshold = threshold
        # self.get_logger().info(f'Hata eşik değeri {threshold} rad olarak ayarlandı')

def main(args=None):
    rclpy.init(args=args)
    
    node = JointStateComparator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
