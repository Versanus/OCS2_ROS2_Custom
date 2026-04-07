#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import csv
import os

class CsvPublisherNode(Node):
    def __init__(self):
        super().__init__('csv_publisher_node')

        self.get_logger().info("CSV Publisher Başlatıldı (Pos+Vel+Eff Aktif).")

        # 1. CSV'deki sütun kök isimleri
        self.csv_joint_names = [
            'FL_HipX', 'FL_HipY', 'FL_Knee',
            'FR_HipX', 'FR_HipY', 'FR_Knee',
            'HL_HipX', 'HL_HipY', 'HL_Knee',
            'HR_HipX', 'HR_HipY', 'HR_Knee'
        ]

        # Ters çevrilmesi istenen eklemlerin listesi
        self.inverted_joints = ['FR_Knee', 'HR_Knee']

        # 2. URDF Mapping
        self.urdf_mapping = {
            'FL_HipX': 'LF_HAA', 'FL_HipY': 'LF_HFE', 'FL_Knee': 'LF_KFE',
            'FR_HipX': 'RF_HAA', 'FR_HipY': 'RF_HFE', 'FR_Knee': 'RF_KFE',
            'HL_HipX': 'LH_HAA', 'HL_HipY': 'LH_HFE', 'HL_Knee': 'LH_KFE',
            'HR_HipX': 'RH_HAA', 'HR_HipY': 'RH_HFE', 'HR_Knee': 'RH_KFE'
        }

        self.urdf_joint_names = []
        for name in self.csv_joint_names:
            if name in self.urdf_mapping:
                self.urdf_joint_names.append(self.urdf_mapping[name])
            else:
                self.get_logger().error(f"Kritik Hata: '{name}' mapping listesinde yok!")

        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmd', 10)

        csv_file_path = 'sttrot_comb.csv'
        self.csv_data = []

        if not os.path.exists(csv_file_path):
            self.get_logger().error(f"CSV dosyası bulunamadı: {os.path.abspath(csv_file_path)}")
            return

        with open(csv_file_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.csv_data.append(row)

        self.current_index = 0
        timer_period = 0.005  # 200 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import csv
import os

class CsvPublisherNode(Node):
    def __init__(self):
        super().__init__('csv_publisher_node')

        self.get_logger().info("CSV Publisher Başlatıldı (Pos+Vel+Eff Aktif).")

        # 1. CSV'deki sütun kök isimleri
        self.csv_joint_names = [
            'FL_HipX', 'FL_HipY', 'FL_Knee',
            'FR_HipX', 'FR_HipY', 'FR_Knee',
            'HL_HipX', 'HL_HipY', 'HL_Knee',
            'HR_HipX', 'HR_HipY', 'HR_Knee'
        ]

        # Ters çevrilmesi istenen eklemlerin listesi
        self.inverted_joints = ['FR_Knee', 'HR_Knee']

        # 2. URDF Mapping
        self.urdf_mapping = {
            'FL_HipX': 'LF_HAA', 'FL_HipY': 'LF_HFE', 'FL_Knee': 'LF_KFE',
            'FR_HipX': 'RF_HAA', 'FR_HipY': 'RF_HFE', 'FR_Knee': 'RF_KFE',
            'HL_HipX': 'LH_HAA', 'HL_HipY': 'LH_HFE', 'HL_Knee': 'LH_KFE',
            'HR_HipX': 'RH_HAA', 'HR_HipY': 'RH_HFE', 'HR_Knee': 'RH_KFE'
        }

        self.urdf_joint_names = []
        for name in self.csv_joint_names:
            if name in self.urdf_mapping:
                self.urdf_joint_names.append(self.urdf_mapping[name])
            else:
                self.get_logger().error(f"Kritik Hata: '{name}' mapping listesinde yok!")

        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, 'joint_cmd', 10)

        csv_file_path = 'trwalk0.15_comb.csv' # walk0.3_comb sttrot_comb stand_comb trot_comb trwalk0.15_comb
        self.csv_data = []

        if not os.path.exists(csv_file_path):
            self.get_logger().error(f"CSV dosyası bulunamadı: {os.path.abspath(csv_file_path)}")
            return

        with open(csv_file_path, 'r') as f:
            reader = csv.DictReader(f)  
            for row in reader:
                self.csv_data.append(row)

        self.current_index = 0
        timer_period = 0.005  # 200 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.current_index >= len(self.csv_data):
            self.get_logger().info("Yayın tamamlandı.")
            self.timer.cancel()
            return

        row = self.csv_data[self.current_index]
        self.current_index += 1
        now = self.get_clock().now().to_msg()

        js_msg = JointState()
        js_msg.header.stamp = now
        js_msg.name = self.urdf_joint_names

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = now
        traj_msg.joint_names = self.csv_joint_names

        for csv_name in self.csv_joint_names:
            # CSV'den ham verileri çek
            pos_val = float(row.get(f"{csv_name}_pos", 0.0))
            vel_val = float(row.get(f"{csv_name}_vel", 0.0))
            eff_val = float(row.get(f"{csv_name}_torque", 0.0))

            # --- TERS ÇEVİRME İŞLEMİ ---
            # Eğer eklem ismi inverted_joints listesindeyse değerleri -1 ile çarp
            if csv_name in self.inverted_joints:
                pos_val *= -1.0
                vel_val *= -1.0
                eff_val *= -1.0

            # JointState listelerine ekle
            js_msg.position.append(-pos_val)
            js_msg.velocity.append(-vel_val)
            js_msg.effort.append(-eff_val)

            # JointTrajectoryPoint
            point = JointTrajectoryPoint()
            point.positions = [pos_val]
            point.velocities = [5.0] # Sabit değerinizi korudum
            point.effort = [12.0] # Sabit değerinizi korudum
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 0
            traj_msg.points.append(point)

        # Yayınla
        self.joint_states_pub.publish(js_msg)
        self.joint_cmd_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CsvPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

def main(args=None):
    rclpy.init(args=args)
    node = CsvPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()