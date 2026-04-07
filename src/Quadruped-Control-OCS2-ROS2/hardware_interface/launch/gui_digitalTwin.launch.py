# gui_digitalTwin.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # XDG_SESSION_TYPE hatasını önlemek için
    env_var = SetEnvironmentVariable(
        name='DISPLAY',
        value=':0'
    )
    
    # Paket dizinini al
    package_dir = get_package_share_directory('urdf_v3_deneme1')
    
    # Script yolunu belirle
    script_path = os.path.join(package_dir, 'scripts', 'digital_twin_gui.py')
    
    # Alternatif yol kontrolü
    if not os.path.exists(script_path):
        # ~/quadv3_ws/src/urdf_v3_deneme1/scripts yolunu kontrol et
        home_dir = os.path.expanduser('~')
        script_path = os.path.join(home_dir, 'quadv3_ws', 'src', 'urdf_v3_deneme1', 'scripts', 'digital_twin_gui.py')
    
    if not os.path.exists(script_path):
        # Install/share yolunu kontrol et
        script_path = os.path.join(package_dir, 'scripts', 'digital_twin_gui.py')
    
    print(f"GUI Script Path: {script_path}")
    
    # GUI scriptini başlat
    gui_process = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen',
        shell=False  # shell=False olarak ayarlayın
    )
    
    return LaunchDescription([
        env_var,
        gui_process
    ])