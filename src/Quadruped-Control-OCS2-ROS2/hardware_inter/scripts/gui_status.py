#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import queue
from std_msgs.msg import String, Bool
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState  # Yeni import

class MotorStatusGUI(Node):
    def __init__(self):
        super().__init__('motor_status_gui')

        # Kalibrasyon publisher'ı ekleyin
        self.calibration_publisher_ = self.create_publisher(Bool, 'leg_calibration', 10)
        
        self.joint_error_queue = queue.Queue()
        self.motor_status_queue = queue.Queue()
        self.joint_state_queue = queue.Queue()  # Yeni queue ekledik

        self.motor_groups = {
            'FRONT LEFT': ['FL_HipX', 'FL_HipY', 'FL_Knee'],
            'FRONT RIGHT': ['FR_HipX', 'FR_HipY', 'FR_Knee'],
            'HIND LEFT': ['HL_HipX', 'HL_HipY', 'HL_Knee'],
            'HIND RIGHT': ['HR_HipX', 'HR_HipY', 'HR_Knee']
        }
        
        self.all_motors = []
        for motors in self.motor_groups.values():
            self.all_motors.extend(motors)

        # Motor durumlarını saklamak için sözlük
        self.motor_states = {motor: "BEKLENİYOR" for motor in self.all_motors}
        
        # Motor detay bilgilerini saklamak için sözlük
        self.motor_details = {motor: {
            'temperature': 'N/A',
            'voltage': 'N/A',
            'current': 'N/A',
            'effort': 'N/A',
            'velocity': 'N/A',
            'position': 'N/A'
        } for motor in self.all_motors}

        # GUI thread
        self.root = None
        self.gui_thread = None
        self.gui_running = True
        self.gui_initialized = False  # GUI başlatıldı mı kontrolü
        self.calibration_armed = False
        self.calibration_arm_timeout_ms = 5000
        self.calibration_reset_job = None

        self.create_subscription(String, 'joint_state_errors', self.joint_error_callback, 10)
        self.create_subscription(DiagnosticArray, 'htdw_status', self.motor_status_callback, 10)
        # Yeni: /htdw_joint_state topic'ine subscribe
        self.create_subscription(JointState, 'htdw_joint_state', self.joint_state_callback, 10)

        self.start_gui()

    # ------------------------------------------------------------
    # ROS CALLBACKS
    # ------------------------------------------------------------
    def joint_error_callback(self, msg):
        self.joint_error_queue.put(msg.data)

    def motor_status_callback(self, msg):
        motor_status_dict = {}
        
        for status_msg in msg.status:
            # Motor ismini al
            motor_name = status_msg.name
            
            # Motor ismini standartlaştır
            motor_name = self.standardize_motor_name(motor_name)
            
            values_dict = {}
            
            for value in status_msg.values:
                values_dict[value.key] = value.value
            
            motor_status_dict[motor_name] = values_dict
        
        self.motor_status_queue.put(motor_status_dict)

    def joint_state_callback(self, msg):
        """Yeni: /htdw_joint_state topic'inden effort, position, velocity bilgilerini al"""
        joint_state_dict = {}
        
        # JointState mesajının yapısı:
        # name: motor isimleri listesi
        # position: pozisyon değerleri listesi
        # velocity: hız değerleri listesi  
        # effort: tork değerleri listesi
        
        for i, motor_name in enumerate(msg.name):
            # Motor ismini standartlaştır
            motor_name = self.standardize_motor_name(motor_name)
            
            # Sadece beklediğimiz motorları işle
            if motor_name in self.all_motors:
                joint_state_dict[motor_name] = {
                    'position': msg.position[i] if i < len(msg.position) else 'N/A',
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 'N/A',
                    'effort': msg.effort[i] if i < len(msg.effort) else 'N/A'
                }
        
        # Queue'ya ekle
        self.joint_state_queue.put(joint_state_dict)

    def standardize_motor_name(self, name):
        """Gelen motor isimlerini standart formata çevir"""
        name = name.strip()
        
        # Doğrudan eşleştirme tablosu
        name_mapping = {
            'FL_Knee': 'FL_Knee',
            'FL_HipY': 'FL_HipY',
            'FL_HipX': 'FL_HipX',
            'FR_Knee': 'FR_Knee',
            'FR_HipY': 'FR_HipY',
            'FR_HipX': 'FR_HipX',
            'HL_Knee': 'HL_Knee',
            'HL_HipY': 'HL_HipY',
            'HL_HipX': 'HL_HipX',
            'HR_Knee': 'HR_Knee',
            'HR_HipY': 'HR_HipY',
            'HR_HipX': 'HR_HipX',
        }
        
        # Büyük/küçük harf duyarsız eşleştirme
        for key, value in name_mapping.items():
            if key.upper() == name.upper():
                return value
        
        # Eğer '_' içeriyorsa formatını düzelt
        if '_' in name:
            parts = name.split('_')
            if len(parts) == 2:
                prefix, motor_type = parts
                
                # Motor tipini standartlaştır
                motor_type = motor_type.upper()
                if motor_type == 'KNEE':
                    motor_type = 'Knee'
                elif motor_type == 'HIPX' or motor_type == 'HIP_X':
                    motor_type = 'HipX'
                elif motor_type == 'HIPY' or motor_type == 'HIP_Y':
                    motor_type = 'HipY'
                
                result = f"{prefix.upper()}_{motor_type}"
                
                if result in self.all_motors:
                    return result
        
        return name

    # ------------------------------------------------------------
    # CALIBRATION BUTTON FUNCTION
    # ------------------------------------------------------------
    def reset_calibration_button(self):
        """Return calibration UI to its safe default state."""
        self.calibration_armed = False
        self.calibration_reset_job = None

        if self.gui_initialized and self.calibration_button.winfo_exists():
            self.calibration_button.config(
                text="LEG CALIBRATION",
                state="normal",
                bg=self.COLORS['calibration_blue'],
                activebackground="#0077B6"
            )

        if self.gui_initialized and self.calibration_info_label.winfo_exists():
            self.calibration_info_label.config(
                text="Requires two clicks within 5 seconds to avoid accidental calibration"
            )

    def arm_calibration(self):
        """Require a second confirmation click before sending calibration."""
        self.calibration_armed = True

        if self.calibration_reset_job is not None:
            self.root.after_cancel(self.calibration_reset_job)

        self.calibration_button.config(
            text="CLICK AGAIN TO CALIBRATE",
            bg=self.COLORS['warning_orange'],
            activebackground="#CC7A00"
        )
        self.calibration_info_label.config(
            text="Calibration is armed. Click again within 5 seconds to send the command."
        )
        self.add_log("Calibration arm enabled. Waiting for confirmation click.")
        self.calibration_reset_job = self.root.after(
            self.calibration_arm_timeout_ms, self.reset_calibration_button)

    def send_calibration_command(self):
        """Send calibration command"""
        if not self.calibration_armed:
            self.arm_calibration()
            return

        if self.calibration_reset_job is not None:
            self.root.after_cancel(self.calibration_reset_job)
            self.calibration_reset_job = None

        self.calibration_armed = False
        msg = Bool()
        msg.data = True
        self.calibration_publisher_.publish(msg)
        self.add_log("Leg calibration komutu gönderildi: True")
        
        # Buton durumunu güncelle
        self.calibration_button.config(
            text="CALIBRATING...",
            state="disabled",
            bg=self.COLORS['inactive_red'],
            activebackground=self.COLORS['inactive_red']
        )
        self.calibration_info_label.config(
            text="Calibration command sent. Button will return to safe mode shortly."
        )
        self.root.after(2000, self.reset_calibration_button)

    # ------------------------------------------------------------
    # GUI START
    # ------------------------------------------------------------
    def start_gui(self):
        """Start GUI on a new thread"""
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

    def run_gui(self):
        self.root = tk.Tk()
        self.root.title("Quadruped Motor Status")
        
        # Optimize edilmiş pencere boyutu
        self.root.geometry("1400x900")  # Daha küçük, yönetilebilir boyut
        
        # RENK PALETİ
        self.COLORS = {
            # Ana renkler
            'bg_dark': '#0A1929',
            'bg_light': '#132F4C',
            'panel_bg': '#1E4976',
            'border': '#2D5A9D',
            
            # Durum renkleri
            'active_green': '#00D4AA',
            'inactive_red': '#FF4D6D',
            'warning_orange': '#FF9E00',
            'waiting_gray': '#7C8BA0',
            'calibration_blue': '#00B4D8',
            
            # Metin renkleri
            'text_light': '#E6F7FF',
            'text_muted': '#B0C4DE',
            'text_dark': '#0A1929',
            
            # Özel motor bilgi renkleri
            'temp_color': '#FF6B8B',
            'volt_color': '#4ECDC4',
            'curr_color': '#45B7D1',
            'effort_color': '#9D65C9',
            'velocity_color': '#2E86AB',
            'position_color': '#00C9B1'
        }
        
        self.root.configure(bg=self.COLORS['bg_dark'])

        style = ttk.Style()
        style.theme_use("clam")
        
        # Ana içerik çerçevesi - padding'ler optimize edildi
        main_container = tk.Frame(self.root, bg=self.COLORS['bg_dark'])
        main_container.pack(fill="both", expand=True, padx=12, pady=12)

        # Sol panel - Motor Durumları
        left_panel = tk.Frame(main_container, bg=self.COLORS['bg_dark'])
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 12))

        # Sağ panel - Kalibrasyon Butonu ve Log Ekranı
        right_panel = tk.Frame(main_container, bg=self.COLORS['bg_dark'], width=450)
        right_panel.pack(side="right", fill="y")

        # ---------------- SOL PANEL: MOTOR DURUMLARI ----------------
        # Motor grid container - 2x2 grid
        motor_grid = tk.Frame(left_panel, bg=self.COLORS['bg_dark'])
        motor_grid.pack(fill="both", expand=True)

        # Grid hücrelerini eşit boyutlandır
        for i in range(2):
            motor_grid.columnconfigure(i, weight=1)
            motor_grid.rowconfigure(i, weight=1)

        self.motor_widgets = {}
        
        # 2x2 grid için bacakları düzenle
        legs = list(self.motor_groups.keys())
        
        for i, leg_name in enumerate(legs):
            row = i // 2
            col = i % 2
            
            # Bacak çerçevesi
            leg_frame = tk.LabelFrame(motor_grid, text=f" {leg_name} ", 
                                      font=("Arial", 13, "bold"),
                                      bg=self.COLORS['panel_bg'], 
                                      fg=self.COLORS['text_light'],
                                      bd=2, 
                                      relief="flat",
                                      labelanchor="n",
                                      padx=8,
                                      pady=8)
            leg_frame.grid(row=row, column=col, sticky="nsew", padx=8, pady=8)
            
            # Bacak çerçevesi içinde grid yapısı
            leg_frame.columnconfigure(0, weight=1)
            
            # Her motor için blok oluştur
            for motor_name in self.motor_groups[leg_name]:
                motor_widget = self.create_motor_block(leg_frame, motor_name)
                motor_widget.pack(fill="x", pady=(0, 6), padx=4)
            
            # Boşluk ekle
            for _ in range(1):
                tk.Frame(leg_frame, height=3, bg=self.COLORS['panel_bg']).pack(fill="x", pady=1)

        # ---------------- SAĞ PANEL: KALİBRASYON ve LOG ----------------
        # Kalibrasyon butonu
        calibration_frame = tk.Frame(right_panel, bg=self.COLORS['panel_bg'], bd=2, relief="flat")
        calibration_frame.pack(fill="x", pady=(0, 12))
        
        # Leg Calibration butonu
        self.calibration_button = tk.Button(
            calibration_frame,
            text="LEG CALIBRATION",
            command=self.send_calibration_command,
            font=("Arial", 14, "bold"),
            bg=self.COLORS['calibration_blue'],
            fg=self.COLORS['text_light'],
            activebackground="#0077B6",
            activeforeground=self.COLORS['text_light'],
            relief="raised",
            bd=3,
            padx=30,
            pady=15,
            cursor="hand2"
        )
        self.calibration_button.pack(padx=25, pady=20)

        # Buton açıklaması
        self.calibration_info_label = tk.Label(
            calibration_frame,
            text="Requires two clicks within 5 seconds to avoid accidental calibration",
            font=("Arial", 10),
            bg=self.COLORS['panel_bg'],
            fg=self.COLORS['text_muted'],
            wraplength=360,
            justify="center"
        )
        self.calibration_info_label.pack(pady=(0, 8))

        # Joint Hataları
        error_title = tk.Label(right_panel, text="JOINT ERROR CONTROL", 
                              font=("Arial", 14, "bold"),
                              bg=self.COLORS['bg_dark'], 
                              fg=self.COLORS['text_light'])
        error_title.pack(anchor="w", pady=(0, 8))

        # Hata çerçevesi
        error_frame = tk.Frame(right_panel, bg=self.COLORS['panel_bg'], bd=2, relief="flat")
        error_frame.pack(fill="both", expand=True, pady=(0, 15))

        # Hata durumu göstergesi
        self.error_status_frame = tk.Frame(error_frame, bg=self.COLORS['panel_bg'])
        self.error_status_frame.pack(fill="x", padx=12, pady=12)
        
        self.error_icon = tk.Label(self.error_status_frame, text="●", 
                                  font=("Arial", 22),
                                  bg=self.COLORS['panel_bg'],
                                  fg=self.COLORS['active_green'])
        self.error_icon.pack(side="left", padx=(0, 12))
        
        self.error_status = tk.Label(self.error_status_frame, text="SYSTEM OK", 
                                    font=("Arial", 13, "bold"),
                                    bg=self.COLORS['panel_bg'],
                                    fg=self.COLORS['active_green'])
        self.error_status.pack(side="left")

        # Hata detayları
        self.error_text = scrolledtext.ScrolledText(error_frame, 
                                                    bg=self.COLORS['bg_light'],
                                                    fg=self.COLORS['text_light'],
                                                    font=("Consolas", 9),
                                                    bd=0,
                                                    relief="flat",
                                                    wrap="word")
        self.error_text.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        
        # Hata sayacı
        error_count_frame = tk.Frame(error_frame, bg=self.COLORS['panel_bg'])
        error_count_frame.pack(fill="x", padx=12, pady=(0, 12))
        
        self.error_count_label = tk.Label(error_count_frame, text="Hata Sayısı: 0",
                                         font=("Arial", 10),
                                         bg=self.COLORS['panel_bg'],
                                         fg=self.COLORS['text_light'])
        self.error_count_label.pack(side="left")
        
        # Log Ekranı
        log_title = tk.Label(right_panel, text="SYSTEM LOG", 
                            font=("Arial", 14, "bold"),
                            bg=self.COLORS['bg_dark'], 
                            fg=self.COLORS['text_light'])
        log_title.pack(anchor="w", pady=(15, 8))

        log_frame = tk.Frame(right_panel, bg=self.COLORS['panel_bg'], bd=2, relief="flat")
        log_frame.pack(fill="both", expand=True)

        self.log_text = scrolledtext.ScrolledText(log_frame, 
                                                  bg=self.COLORS['bg_light'],
                                                  fg=self.COLORS['text_light'],
                                                  font=("Consolas", 8),
                                                  bd=0,
                                                  relief="flat")
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

        # GUI başlatıldı olarak işaretle
        self.gui_initialized = True
        
        self.update_gui()
        self.root.mainloop()
        
        # Pencere kapatıldığında
        self.gui_running = False
        self.gui_initialized = False

    def create_motor_block(self, parent_frame, motor_name):
        """Her motor için bir durum bloğu oluştur"""
        # Ana motor çerçevesi - optimize edilmiş boyut
        motor_frame = tk.Frame(parent_frame, 
                              bg=self.COLORS['bg_light'],
                              bd=1,
                              relief="flat",
                              padx=10,
                              pady=8)
        
        # Motor başlığı
        motor_header = tk.Frame(motor_frame, bg=self.COLORS['bg_light'])
        motor_header.pack(fill="x", pady=(0, 6))
        
        # Motor adı
        motor_label = tk.Label(motor_header, 
                              text=motor_name.replace('_', ' '),
                              font=("Arial", 11, "bold"),
                              bg=self.COLORS['bg_light'],
                              fg=self.COLORS['text_light'],
                              anchor="w")
        motor_label.pack(side="left", fill="x", expand=True)
        
        # Durum göstergesi
        status_frame = tk.Frame(motor_header, bg=self.COLORS['bg_light'])
        status_frame.pack(side="right")
        
        status_dot = tk.Label(status_frame, text="●", 
                             font=("Arial", 14),
                             bg=self.COLORS['bg_light'],
                             fg=self.COLORS['waiting_gray'])
        status_dot.pack(side="left", padx=(0, 6))
        
        status_text = tk.Label(status_frame, text="WAITING",
                              font=("Arial", 9, "bold"),
                              bg=self.COLORS['bg_light'],
                              fg=self.COLORS['waiting_gray'])
        status_text.pack(side="left")
        
        # Detaylar için 2 satırlık grid
        details_frame = tk.Frame(motor_frame, bg=self.COLORS['bg_light'])
        details_frame.pack(fill="x")
        
        # 1. Satır: Temel bilgiler
        row1_frame = tk.Frame(details_frame, bg=self.COLORS['bg_light'])
        row1_frame.pack(fill="x", pady=(0, 4))
        
        # Sıcaklık
        temp_frame = tk.Frame(row1_frame, bg=self.COLORS['bg_light'])
        temp_frame.pack(side="left", fill="x", expand=True, padx=2)
        
        tk.Label(temp_frame, text="TEMPERATURE", 
                font=("Arial", 7, "bold"),
                bg=self.COLORS['bg_light'],
                fg=self.COLORS['temp_color']).pack(anchor="center")
        
        temp_value = tk.Label(temp_frame, text="N/A",
                             font=("Arial", 9, "bold"),
                             bg=self.COLORS['bg_light'],
                             fg=self.COLORS['temp_color'])
        temp_value.pack(anchor="center")
        
        # Voltaj
        volt_frame = tk.Frame(row1_frame, bg=self.COLORS['bg_light'])
        volt_frame.pack(side="left", fill="x", expand=True, padx=2)
        
        tk.Label(volt_frame, text="VOLTAJ", 
                font=("Arial", 7, "bold"),
                bg=self.COLORS['bg_light'],
                fg=self.COLORS['volt_color']).pack(anchor="center")
        
        volt_value = tk.Label(volt_frame, text="N/A",
                             font=("Arial", 9, "bold"),
                             bg=self.COLORS['bg_light'],
                             fg=self.COLORS['volt_color'])
        volt_value.pack(anchor="center")
        
        # Akım
        curr_frame = tk.Frame(row1_frame, bg=self.COLORS['bg_light'])
        curr_frame.pack(side="left", fill="x", expand=True, padx=2)
        
        tk.Label(curr_frame, text="CURRENT", 
                font=("Arial", 7, "bold"),
                bg=self.COLORS['bg_light'],
                fg=self.COLORS['curr_color']).pack(anchor="center")
        
        curr_value = tk.Label(curr_frame, text="N/A",
                             font=("Arial", 9, "bold"),
                             bg=self.COLORS['bg_light'],
                             fg=self.COLORS['curr_color'])
        curr_value.pack(anchor="center")
        
        # 2. Satır: Effort, Velocity, Position
        row2_frame = tk.Frame(details_frame, bg=self.COLORS['bg_light'])
        row2_frame.pack(fill="x", pady=(4, 0))
        
        # Effort (Tork)
        effort_frame = tk.Frame(row2_frame, bg=self.COLORS['bg_light'])
        effort_frame.pack(side="left", fill="x", expand=True, padx=2)
        
        tk.Label(effort_frame, text="EFFORT", 
                font=("Arial", 7, "bold"),
                bg=self.COLORS['bg_light'],
                fg=self.COLORS['effort_color']).pack(anchor="center")
        
        effort_value = tk.Label(effort_frame, text="N/A",
                               font=("Arial", 9, "bold"),
                               bg=self.COLORS['bg_light'],
                               fg=self.COLORS['effort_color'])
        effort_value.pack(anchor="center")
        
        # Velocity (Hız)
        velocity_frame = tk.Frame(row2_frame, bg=self.COLORS['bg_light'])
        velocity_frame.pack(side="left", fill="x", expand=True, padx=2)
        
        tk.Label(velocity_frame, text="VELOCITY", 
                font=("Arial", 7, "bold"),
                bg=self.COLORS['bg_light'],
                fg=self.COLORS['velocity_color']).pack(anchor="center")
        
        velocity_value = tk.Label(velocity_frame, text="N/A",
                                 font=("Arial", 9, "bold"),
                                 bg=self.COLORS['bg_light'],
                                 fg=self.COLORS['velocity_color'])
        velocity_value.pack(anchor="center")
        
        # Position
        position_frame = tk.Frame(row2_frame, bg=self.COLORS['bg_light'])
        position_frame.pack(side="left", fill="x", expand=True, padx=2)
        
        tk.Label(position_frame, text="POSITION", 
                font=("Arial", 7, "bold"),
                bg=self.COLORS['bg_light'],
                fg=self.COLORS['position_color']).pack(anchor="center")
        
        position_value = tk.Label(position_frame, text="N/A",
                                 font=("Arial", 9, "bold"),
                                 bg=self.COLORS['bg_light'],
                                 fg=self.COLORS['position_color'])
        position_value.pack(anchor="center")
        
        # Widget'ları sakla
        self.motor_widgets[motor_name] = {
            'frame': motor_frame,
            'status_dot': status_dot,
            'status_text': status_text,
            'temp_value': temp_value,
            'volt_value': volt_value,
            'curr_value': curr_value,
            'effort_value': effort_value,
            'velocity_value': velocity_value,
            'position_value': position_value
        }
        
        return motor_frame

    # ------------------------------------------------------------
    # GUI UPDATE FUNCTIONS
    # ------------------------------------------------------------
    def update_gui(self):
        try:
            while True:
                motor_status_dict = self.motor_status_queue.get_nowait()
                self.update_motor_status(motor_status_dict)
        except queue.Empty:
            pass

        try:
            while True:
                error_data = self.joint_error_queue.get_nowait()
                self.update_joint_errors(error_data)
        except queue.Empty:
            pass

        try:
            while True:
                joint_state_dict = self.joint_state_queue.get_nowait()
                self.update_joint_state_values(joint_state_dict)
        except queue.Empty:
            pass

        if self.gui_running:
            self.root.after(100, self.update_gui)

    def update_motor_status(self, motor_status_dict):
        if not motor_status_dict:
            return
            
        inactive_count = 0
        active_count = 0
        unknown_count = 0
        
        # Gelen motor verilerini logla
        self.add_log(f"Motor durumu güncellendi: {len(motor_status_dict)} motor")
        
        # Gelen verileri detaylı logla
        for motor_name, values in motor_status_dict.items():
            self.add_log(f"Motor {motor_name}: is_active='{values.get('is_active', 'N/A')}'")
        
        # Öncelikle tüm motorları bekliyor durumuna getir
        for motor_name in self.all_motors:
            if motor_name in self.motor_widgets:
                widgets = self.motor_widgets[motor_name]
                
                # Motor durumu geldi mi?
                if motor_name in motor_status_dict:
                    values = motor_status_dict[motor_name]
                    
                    # is_active değerini al ve büyük harfe çevir
                    is_active_value = values.get('is_active', 'UNKNOWN')
                    status_str = str(is_active_value).upper().strip()
                    
                    # Değerleri al
                    temperature = values.get('temperature', 'N/A')
                    voltage = values.get('voltage', 'N/A')
                    current = values.get('current', 'N/A')
                    
                    # DÜZELTME: Tam eşleşme kullan, "in" operatörü kullanma!
                    if status_str == 'ACTIVE':
                        color = self.COLORS['active_green']
                        status_text = "AKTİF"
                        frame_bg = '#1E3A28'
                        active_count += 1
                    elif status_str == 'INACTIVE':
                        color = self.COLORS['inactive_red']
                        status_text = "İNAKTİF"
                        frame_bg = '#3A1E1E'
                        inactive_count += 1
                    else:
                        # Bilinmeyen durum
                        color = self.COLORS['warning_orange']
                        status_text = f"BEKLENİYOR ({status_str[:10]})"
                        frame_bg = self.COLORS['bg_light']
                        unknown_count += 1
                    
                    # Widget'ları güncelle
                    widgets['frame'].config(bg=frame_bg)
                    widgets['status_dot'].config(bg=frame_bg, fg=color)
                    widgets['status_text'].config(bg=frame_bg, fg=color, text=status_text)
                    
                    # Değerleri ayarla (N/A olanları gizle)
                    temp_val = f"{temperature}°C" if temperature != 'N/A' else "N/A"
                    volt_val = f"{voltage}V" if voltage != 'N/A' else "N/A"
                    curr_val = f"{current}A" if current != 'N/A' else "N/A"
                    
                    widgets['temp_value'].config(bg=frame_bg, text=temp_val)
                    widgets['volt_value'].config(bg=frame_bg, text=volt_val)
                    widgets['curr_value'].config(bg=frame_bg, text=curr_val)
                    
                    # Tüm child widget'ların arka planını güncelle
                    self.update_widget_bg(widgets['frame'], frame_bg)
                else:
                    # Motor verisi gelmedi
                    widgets['frame'].config(bg=self.COLORS['bg_light'])
                    widgets['status_dot'].config(bg=self.COLORS['bg_light'], 
                                              fg=self.COLORS['waiting_gray'])
                    widgets['status_text'].config(bg=self.COLORS['bg_light'], 
                                                fg=self.COLORS['waiting_gray'], 
                                                text="VERİ YOK")
                    self.update_widget_bg(widgets['frame'], self.COLORS['bg_light'])
                    unknown_count += 1
        
        # Log ekranına durum yaz
        status_summary = f"Motor güncellendi: {active_count} aktif, {inactive_count} inaktif, {unknown_count} bilinmiyor/bekliyor"
        self.add_log(status_summary)

    def update_joint_state_values(self, joint_state_dict):
        """Yeni: /htdw_joint_state topic'inden gelen effort, position, velocity değerlerini güncelle"""
        if not joint_state_dict:
            return
            
        updated_count = 0
        
        for motor_name in self.all_motors:
            if motor_name in self.motor_widgets and motor_name in joint_state_dict:
                widgets = self.motor_widgets[motor_name]
                values = joint_state_dict[motor_name]
                
                # Değerleri formatla
                effort = values.get('effort', 'N/A')
                velocity = values.get('velocity', 'N/A')
                position = values.get('position', 'N/A')
                
                # Değerleri widget'lara ata
                if effort != 'N/A':
                    # Ondalık hassasiyetini ayarla
                    effort_val = f"{float(effort):.3f}Nm" 
                    widgets['effort_value'].config(text=effort_val)
                
                if velocity != 'N/A':
                    velocity_val = f"{float(velocity):.6f}rad/s"
                    widgets['velocity_value'].config(text=velocity_val)
                
                if position != 'N/A':
                    position_val = f"{float(position):.6f}rad"
                    widgets['position_value'].config(text=position_val)
                
                updated_count += 1
        
        if updated_count > 0:
            self.add_log(f"Joint state güncellendi: {updated_count} motor")

    def update_widget_bg(self, widget, bg_color):
        """Bir widget ve tüm child'larının arka planını güncelle"""
        try:
            widget.config(bg=bg_color)
            for child in widget.winfo_children():
                if isinstance(child, (tk.Frame, tk.Label)):
                    child.config(bg=bg_color)
                    # Recursive olarak tüm child'ları güncelle
                    self.update_widget_bg(child, bg_color)
        except:
            pass

    def update_joint_errors(self, error_data):
        self.error_text.delete("1.0", tk.END)
        
        if error_data == "No error" or error_data == "" or error_data is None:
            self.error_text.insert(tk.END, "✓ Tüm jointler uyumlu. Sistem normal çalışıyor.\n\n")
            self.error_icon.config(fg=self.COLORS['active_green'], text="✓")
            self.error_status.config(text="SYSTEM OK", fg=self.COLORS['active_green'])
            self.error_count_label.config(text="Hata Sayısı: 0")
            return
        
        lines = error_data.split("\n")
        error_lines = [line for line in lines if line.strip()]
        
        # Hata durumunu güncelle
        if error_lines:
            self.error_icon.config(fg=self.COLORS['inactive_red'], text="⚠")
            self.error_status.config(text=f"{len(error_lines)} HATA TESPİT EDİLDİ", 
                                   fg=self.COLORS['inactive_red'])
            self.error_count_label.config(text=f"Hata Sayısı: {len(error_lines)}")
            
            # Hataları formatlı şekilde göster
            self.error_text.insert(tk.END, f"Toplam {len(error_lines)} hata tespit edildi:\n")
            self.error_text.insert(tk.END, "="*40 + "\n\n")
            
            for i, line in enumerate(error_lines, 1):
                if "Fark=" in line or "Error" in line or "Hata" in line:
                    self.error_text.insert(tk.END, f"[{i}] ⚠ {line}\n", "warning")
                else:
                    self.error_text.insert(tk.END, f"[{i}] {line}\n")
                
                self.error_text.insert(tk.END, "-"*25 + "\n")
        
        # Tag konfigürasyonu
        self.error_text.tag_config("warning", 
                                 foreground=self.COLORS['inactive_red'], 
                                 font=("Consolas", 9, "bold"))
        self.error_text.tag_config("normal", 
                                 foreground=self.COLORS['text_light'])
        
        # Scroll'u en üste al
        self.error_text.see("1.0")

    # ------------------------------------------------------------
    # LOGGING
    # ------------------------------------------------------------
    def add_log(self, msg):
        # GUI henüz hazır değilse loglama yapma
        if not self.gui_initialized or not hasattr(self, 'log_text'):
            # Sadece konsola yaz (debug için)
            print(f"[GUI NOT READY] {msg}")
            return
            
        from datetime import datetime
        t = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Log formatı
        log_entry = f"[{t}] {msg}\n"
        self.log_text.insert(tk.END, log_entry)
        
        # Log'u renklendirme
        if "hata" in msg.lower() or "error" in msg.lower() or "inaktif" in msg.lower():
            self.log_text.tag_add("error", f"end-2l linestart", f"end-2l lineend")
        elif "aktif" in msg.lower() or "active" in msg.lower():
            self.log_text.tag_add("success", f"end-2l linestart", f"end-2l lineend")
        elif "gelen" in msg.lower() or "güncellendi" in msg.lower():
            self.log_text.tag_add("info", f"end-2l linestart", f"end-2l lineend")
        elif "calibration" in msg.lower() or "kalibrasyon" in msg.lower():
            self.log_text.tag_add("calibration", f"end-2l linestart", f"end-2l lineend")
        else:
            self.log_text.tag_add("normal", f"end-2l linestart", f"end-2l lineend")
        
        # Tag renkleri
        self.log_text.tag_config("error", foreground=self.COLORS['inactive_red'])
        self.log_text.tag_config("success", foreground=self.COLORS['active_green'])
        self.log_text.tag_config("info", foreground=self.COLORS['calibration_blue'])
        self.log_text.tag_config("calibration", foreground=self.COLORS['position_color'])
        self.log_text.tag_config("normal", foreground=self.COLORS['text_muted'])
        
        # En son log'u göster
        self.log_text.see(tk.END)
        
        # Log limiti (son 100 mesaj)
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 100:
            self.log_text.delete(1.0, 2.0)


def main(args=None):
    rclpy.init(args=args)
    gui = MotorStatusGUI()

    spin_thread = threading.Thread(target=rclpy.spin, args=(gui,), daemon=True)
    spin_thread.start()

    gui.gui_thread.join()

    gui.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
