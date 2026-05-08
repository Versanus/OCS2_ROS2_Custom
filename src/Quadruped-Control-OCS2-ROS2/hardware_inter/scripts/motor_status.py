#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus

class Bcolors:
    """Renkli terminal çıktıları için ANSI renk kodları"""
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    Red = '\033[1;31;40m'

class MotorMonitor(Node):
    def __init__(self):
        super().__init__('motor_monitor')
        
        # Abone ol: topic adı ve mesaj tipi
        self.subscription = self.create_subscription(
            DiagnosticStatus,
            'htdw_status',  # Topic adı
            self.status_callback,
            10
        )
        
        # self.get_logger().info(f"{Bcolors.OKCYAN}Motor durum izleyici başlatıldı...{Bcolors.ENDC}")
        
        # Dokümandaki isimlendirmeye göre Türkçe motor/bacak isimleri
        self.motor_display_names = {
            # Ön Sol Bacak (FL)
            'FL_HipX': 'ÖN SOL',
            'FL_HipY': 'ÖN SOL',
            'FL_Knee': 'ÖN SOL',
            
            # Ön Sağ Bacak (FR)
            'FR_HipX': 'ÖN SAĞ',
            'FR_HipY': 'ÖN SAĞ',
            'FR_Knee': 'ÖN SAĞ',
            
            # Arka Sol Bacak (HL)
            'HL_HipX': 'ARKA SOL',
            'HL_HipY': 'ARKA SOL',
            'HL_Knee': 'ARKA SOL',
            
            # Arka Sağ Bacak (HR)
            'HR_HipX': 'ARKA SAĞ',
            'HR_HipY': 'ARKA SAĞ',
            'HR_Knee': 'ARKA SAĞ'
        }
        
        # Tüm motor isimleri listesi
        self.all_motors = list(self.motor_display_names.keys())
        
    def status_callback(self, msg):
        """DiagnosticStatus mesajını işle"""
        
        # Mesajdaki motor durumlarını topla
        motor_status = {}
        for kv in msg.values:
            motor_status[kv.key] = kv.value
        
        # Tüm motorları kontrol et
        inactive_motors = []
        
        for motor_key in self.all_motors:
            if motor_key in motor_status:
                if motor_status[motor_key] == 'INACTIVE':
                    inactive_motors.append(motor_key)
            else:
                # Motor mesajda yoksa INACTIVE kabul et
                inactive_motors.append(motor_key)
        
        # Log bilgisi oluştur - RENKLİ
        if len(inactive_motors) == 0:
            # Tüm motorlar aktif - YEŞİL
            success_msg = f"{Bcolors.OKGREEN}{Bcolors.BOLD}✓ TÜM MOTORLAR AKTİF{Bcolors.ENDC}{Bcolors.OKGREEN}: Ön Sol, Ön Sağ, Arka Sol, Arka Sağ bacakların tüm eklemleri çalışıyor.{Bcolors.ENDC}"
            # self.get_logger().info(success_msg)
        elif len(inactive_motors) == len(self.all_motors):
            # Hiç motor aktif değil - KIRMIZI
            error_msg = f"{Bcolors.FAIL}{Bcolors.BOLD}✗ HİÇBİR MOTOR AKTİF DEĞİL{Bcolors.ENDC}{Bcolors.FAIL}: Tüm motorlar INACTIVE durumda{Bcolors.ENDC}"
            # self.get_logger().error(error_msg)
        else:
            # Bazı motorlar aktif değil - SARI uyarı renkli
            inactive_display = []
            for motor_key in inactive_motors:
                if motor_key in self.motor_display_names:
                    inactive_display.append(self.motor_display_names[motor_key])
            
            inactive_list = ", ".join(inactive_display)
            warning_msg = f"{Bcolors.WARNING}{Bcolors.BOLD}⚠ AKTİF OLMAYAN MOTORLAR{Bcolors.ENDC}{Bcolors.WARNING}: {inactive_list}{Bcolors.ENDC}"
            # self.get_logger().warn(warning_msg)
            
            # Ek olarak: Aktif olan motor sayısı (opsiyonel)
            active_count = len(self.all_motors) - len(inactive_motors)
            total_count = len(self.all_motors)
            status_msg = f"{Bcolors.OKCYAN}[{active_count}/{total_count}] motor aktif{Bcolors.ENDC}"
            # self.get_logger().info(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    motor_monitor = MotorMonitor()
    
    try:
        rclpy.spin(motor_monitor)
    except KeyboardInterrupt:
        stop_msg = f"{Bcolors.OKBLUE}Motor izleyici durduruluyor...{Bcolors.ENDC}"
        motor_monitor.get_logger().info(stop_msg)
    finally:
        motor_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()