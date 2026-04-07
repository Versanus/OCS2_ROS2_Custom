#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
        #               OFFSET DEĞERLERİ
        # ==========================================
        self.referance_offsets = {
            'FL_HipX': -0.322415828704834,
            'FL_HipY': 0.4309038817882538,
            'FL_Knee': -0.4077022075653076,
            'FR_HipX': -0.33759668469429016,
            'FR_HipY': 0.44108203053474426,
            'FR_Knee': 0.42747020721435547,
            'HL_HipX': -0.32241925597190857,
            'HL_HipY': 0.43023791909217834,
            'HL_Knee': -0.4072657823562622,
            'HR_HipX': -0.3375966548919678,
            'HR_HipY': 0.44040337204933167,
            'HR_Knee': 0.4270230531692505
        }

        self.home2Controller_offsets = {
            'FL_HipX': 0.322415828704834,
            'FL_HipY': 0.2509038817882538,
            'FL_Knee': -0.4077022075653076,
            'FR_HipX': -0.33759668469429016,
            'FR_HipY': 0.25108203053474426,
            'FR_Knee': -0.42747020721435547,
            'HL_HipX': 0.32241925597190857,
            'HL_HipY': 0.25023791909217834,
            'HL_Knee': -0.4072657823562622,
            'HR_HipX': -0.3375966548919678,
            'HR_HipY': 0.25040337204933167,
            'HR_Knee': -0.4270230531692505
        }

        self.calibration2Home_offsets = {
            'FL_HipX': 0.0,
            'FL_HipY': -1.050324412,  
            'FL_Knee': 1.9584364492350666,   
            'FR_HipX': 0.0,
            'FR_HipY': 1.050324412,  
            'FR_Knee': -1.95584364492350666,   
            'HL_HipX': 0.0,
            'HL_HipY': -0.950324412,  
            'HL_Knee': 1.9584364492350666,   
            'HR_HipX': 0.0,
            'HR_HipY': 0.950324412,  
            'HR_Knee': -1.9584364492350666    
        }

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
            # SİMÜLASYON -> GERÇEK ROBOT
            self.sim_sub = self.create_subscription(
                JointState, 
                'joint_states', 
                self.sim_to_htdw_callback, 
                10)
            # Gerçek motor donanımına trajectory mesajı atıyoruz
            self.htdw_cmd_pub = self.create_publisher(JointTrajectory, 'htdw_joint_cmd', 10)

    def htdw_to_sim_callback(self, msg: JointState):
        """[MODE 1] Gerçek motordan gelen veriyi RViz (Simülasyon) tarafına yansıt"""
        sim_msg = JointState()
        sim_msg.header.stamp = self.get_clock().now().to_msg()
        sim_msg.name = msg.name
        sim_msg.position = msg.position
        if msg.velocity: sim_msg.velocity = msg.velocity
        if msg.effort: sim_msg.effort = msg.effort
        
        self.sim_pub.publish(sim_msg)


    def sim_to_htdw_callback(self, msg: JointState):
        """[MODE 2] Simülasyondan gelen veriyi hesapla, rate-limit uygula ve donanıma gönder"""
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
            # 1. POZİSYON DEĞERİNİ BELİRLE VE YÖNÜNÜ AYARLA
            # ---------------------------------------------------
            if joint_name in ['HL_HipX', 'FL_HipY', 'FL_Knee', 'HL_HipY', 'HL_Knee', 'HR_HipX']:
                joint_position = msg.position[i]
            else:
                joint_position = -msg.position[i]
            
            # ---------------------------------------------------
            # 2. MOTORLAR İÇİN OFFSETLERİ UYGULA
            # ---------------------------------------------------
            if joint_name in ['HL_HipX', 'FL_HipY', 'FL_Knee', 'HL_HipY', 'HL_Knee', 'HR_HipX']:
                pos = joint_position + self.calibration2Home_offsets.get(joint_name, 0.0) + self.home2Controller_offsets.get(joint_name, 0.0)
            elif joint_name in self.calibration2Home_offsets:
                # Diğer motorlar için offset uygula
                pos = joint_position + self.calibration2Home_offsets[joint_name] - self.home2Controller_offsets.get(joint_name, 0.0)
            else:
                pos = joint_position   

            # ---------------------------------------------------
            # 3. KATSAYI İÇİN REFERANS NOKTAYI HESAPLA
            # ---------------------------------------------------
            if joint_name in self.calibration2Home_offsets:
                # DİKKAT: Gönderdiğin koddaki listeye göre FL_HipX ile başlıyor
                if joint_name in ['FL_HipX', 'FL_HipY', 'FL_Knee', 'HL_HipY', 'HL_Knee', 'HR_HipX']:
                    reference = self.referance_offsets.get(joint_name, 0.0) + self.calibration2Home_offsets[joint_name] + self.home2Controller_offsets.get(joint_name, 0.0)
                else:
                    reference = self.referance_offsets.get(joint_name, 0.0) + self.calibration2Home_offsets[joint_name] - self.home2Controller_offsets.get(joint_name, 0.0)
            else:
                reference = 0.0

            # ---------------------------------------------------
            # 4. KNEE MOTORLARI İÇİN SAPMA (DEVIATION) ÇARPANI
            # ---------------------------------------------------
            if 'Knee' in joint_name:
                deviation = pos - reference
                pos = reference + (deviation * 1.5)

            # ---------------------------------------------------
            # 5. TRAJECTORY POINT MESAJINI DOLDUR
            # ---------------------------------------------------
            # Her point sadece 1 eleman içerir
            point.positions = [float(pos)]
            point.velocities = [2.0]  
            point.effort = [12.0]
            point.accelerations = []

            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 0

            traj_msg.points.append(point)

        # Mesajı donanıma yayınla ve son yayın zamanını güncelle
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