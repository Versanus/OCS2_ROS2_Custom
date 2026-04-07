#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from legged_msgs.msg import JointControlData

class DigitalTwinBridge(Node):
    def __init__(self):
        super().__init__('digital_twin_bridge')

        # --- MOD SEÇİMİ PARMETRESİ ---
        self.declare_parameter('mode', 'real2sim')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.get_logger().info(f"Digital Twin Bridge Başlatıldı. Aktif Mod: {self.mode.upper()}")

        # Ortak Eklem İsimleri
        self.joint_names = [
            'FL_HipX', 'FL_HipY', 'FL_Knee',
            'FR_HipX', 'FR_HipY', 'FR_Knee',
            'HL_HipX', 'HL_HipY', 'HL_Knee',
            'HR_HipX', 'HR_HipY', 'HR_Knee'
        ]

        # ==========================================
        #          PUBLISHER & SUBSCRIBER
        # ==========================================
        if self.mode == 'real2sim':
            # GERÇEK ROBOT -> SİMÜLASYON
            self.htdw_sub = self.create_subscription(
                JointState, 
                'htdw_joint_state', 
                self.htdw_to_sim_callback, 
                10)
            self.sim_pub = self.create_publisher(JointState, 'joint_states', 10)

        elif self.mode == 'sim2real':
            # SİMÜLASYON -> GERÇEK ROBOT
            self.sim_sub = self.create_subscription(
                JointState, 
                'joint_states', 
                self.sim_to_htdw_callback, 
                10)
            # Gerçek motor donanımına trajectory mesajı atıyoruz
            self.htdw_cmd_pub = self.create_publisher(JointTrajectory, 'htdw_joint_cmd', 10)

        elif self.mode == 'controller':
            # MPC KONTROLÖR -> GERÇEK ROBOT
            # JointControlData tipinde mesaj dinle
            self.control_sub = self.create_subscription(
                JointControlData, 
                'joint_control_data', 
                self.control_to_htdw_callback, 
                10)
            # Gerçek motor donanımına trajectory mesajı atıyoruz
            self.htdw_cmd_pub = self.create_publisher(JointTrajectory, 'htdw_joint_cmd', 10)
            self.sim_pub = self.create_publisher(JointState, 'joint_states', 10)
    def htdw_to_sim_callback(self, msg: JointState):
        """[MODE 1] Gerçek motordan gelen veriyi RViz (Simülasyon) tarafına yansıt"""
        sim_msg = JointState()
        sim_msg.header.stamp = self.get_clock().now().to_msg()
        sim_msg.name = msg.name
        sim_msg.position = msg.position
        if msg.velocity: sim_msg.velocity = msg.velocity
        if msg.effort: sim_msg.effort = msg.effort
        
        self.sim_pub.publish(sim_msg)

    def control_to_htdw_callback(self, msg: JointControlData):
        """[MODE controller] MPC kontrolörden gelen JointControlData mesajını donanıma gönder"""
        current_time = self.get_clock().now()

        # JointControlData'dan pozisyonları al (joint_position)
        positions = msg.joint_position
        
        # Eğer pozisyon verisi yoksa çık
        if len(positions) != len(self.joint_names):
            self.get_logger().warn(f"Beklenen joint sayısı: {len(self.joint_names)}, Gelen: {len(positions)}")
            return

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = current_time.to_msg()
        traj_msg.header.frame_id = ''
        traj_msg.joint_names = self.joint_names

        for i, joint_name in enumerate(self.joint_names):
            # Güvenlik kontrolü
            if i >= len(positions):
                continue

            point = JointTrajectoryPoint()  

            # ---------------------------------------------------
            # POZİSYON DEĞERİNİ DOĞRUDAN KULLAN
            # ---------------------------------------------------
            pos = float(positions[i])

            # ---------------------------------------------------
            # TRAJECTORY POINT MESAJINI DOLDUR
            # ---------------------------------------------------
            point.positions = [pos]
            
            # MPC'den gelen hız ve tork verilerini kullan
            if len(msg.joint_velocity) > i:
                point.velocities = [float(msg.joint_velocity[i])]
            else:
                point.velocities = [0.0]  
            
            if len(msg.joint_torque) > i:
                point.effort = [float(msg.joint_torque[i])]
            else:
                point.effort = [0.0]
            
            point.accelerations = []

            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 0

            traj_msg.points.append(point)

        # Mesajı donanıma yayınla
        self.htdw_cmd_pub.publish(traj_msg)
        self.get_logger().debug(f"MPC kontrol mesajı gönderildi. Mod: {msg.actuator_mode}")

    def sim_to_htdw_callback(self, msg: JointState):
        """[MODE 2] Simülasyondan gelen veriyi doğrudan donanıma gönder (OFFSETSİZ)"""
        current_time = self.get_clock().now()

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = current_time.to_msg()
        traj_msg.header.frame_id = ''
        traj_msg.joint_names = msg.name

        for i, joint_name in enumerate(msg.name):
            # Güvenlik kontrolü: Gelen position dizisi joint sayısından kısaysa atla
            if i >= len(msg.position):
                continue

            point = JointTrajectoryPoint()  

            # ---------------------------------------------------
            # POZİSYON DEĞERİNİ DOĞRUDAN KULLAN (OFFSET YOK)
            # ---------------------------------------------------
            pos = float(msg.position[i])

            # ---------------------------------------------------
            # TRAJECTORY POINT MESAJINI DOLDUR
            # ---------------------------------------------------
            point.positions = [pos]
            point.velocities = [0.0]  
            point.effort = [0.0]
            point.accelerations = []

            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 0

            traj_msg.points.append(point)

        # Mesajı donanıma yayınla
        self.htdw_cmd_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    bridge_node = DigitalTwinBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
