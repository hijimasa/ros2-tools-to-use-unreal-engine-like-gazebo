import os
import socket
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory
import shutil
import xml.etree.ElementTree as ET

import sys
sys.path.append(r'/home/ue4/UnrealEngine/Engine/Plugins/Experimental/PythonScriptPlugin/Content/Python')
import remote_execution as remote
import time

def executeCommand(command):
    remote_config = remote.RemoteExecutionConfig()
    remote_config.multicast_bind_address = "0.0.0.0"
    remote_exec = remote.RemoteExecution(config=remote_config)
    remote_exec.start()
    timeout_count = 0
    while not remote_exec.remote_nodes:
        time.sleep(0.1)
        timeout_count += 1
        if timeout_count > 100:
            remote_exec.stop()
            return None
    remote_exec.open_command_connection(remote_exec.remote_nodes[0])
    rec = remote_exec.run_command(command, exec_mode=remote.MODE_EXEC_STATEMENT)
    if rec['success'] == True:
        # Stop session 
        remote_exec.stop()
        return rec['result']
    # Don't leaving session open if can't run
    remote_exec.stop()
    return None

def change_file_extension(file_path, new_extension):
    # ファイルパスを拡張子とそれ以外に分割
    base, _ = os.path.splitext(file_path)
    # 新しい拡張子を追加
    if not new_extension.startswith("."):
        new_extension = "." + new_extension
    return base + new_extension

class SimLancher(Node):
    def __init__(self):
        super().__init__('sim_launcher')

        self.declare_parameter('urdf_path', '')
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        if urdf_path == '':
            return
        urdf_file_name = os.path.basename(urdf_path)
        self.declare_parameter('x', 0.0)
        robot_x = self.get_parameter('x').get_parameter_value().double_value
        self.declare_parameter('y', 0.0)
        robot_y = self.get_parameter('y').get_parameter_value().double_value
        self.declare_parameter('z', 0.0)
        robot_z = self.get_parameter('z').get_parameter_value().double_value
        self.declare_parameter('R', 0.0)
        robot_roll = self.get_parameter('R').get_parameter_value().double_value
        self.declare_parameter('P', 0.0)
        robot_pitch = self.get_parameter('P').get_parameter_value().double_value
        self.declare_parameter('Y', 0.0)
        robot_yaw = self.get_parameter('Y').get_parameter_value().double_value
        self.declare_parameter('fixed', False)
        robot_fixed = self.get_parameter('fixed').get_parameter_value().bool_value
        if robot_fixed:
            robot_fixed_string = "true"
        else:
            robot_fixed_string = "false"

        self.get_logger().info("command start")
        
        # Convert to FBX file
        fbx_path = change_file_extension(urdf_path, ".fbx")
        convert_script_path = os.path.join(
                    get_package_share_directory('unreal_engine_ros2_scripts'), 'convert_urdf_to_fbx.py')
        convert_command = ["/usr/local/blender/blender", "--background", "--python", convert_script_path, "--", urdf_path, fbx_path]
        print(convert_command)
        self.get_logger().info("command start")
        self.convert_proc = subprocess.Popen(convert_command)
        self.convert_proc.wait()
        if not self.convert_proc.stdout == None:
            lines = self.convert_proc.stdout.read()
            for line in lines:
                print(line)
        self.get_logger().info("command end")

        #self.send_urdf_import_settings("URDF_IMPORT " + unity_project_path + "/Assets/Urdf/" + package_name + "/" + urdf_file_name + " " + str(robot_x) + " " + str(robot_y) + " " + str(robot_z) + " " + str(robot_roll) + " " + str(robot_pitch) + " " + str(robot_yaw) + " " + robot_fixed_string)
        import_script_path = os.path.join(
                    get_package_share_directory('unreal_engine_ros2_scripts'), 'import_fbx_and_create_blueprint.py')
        with open(import_script_path, "r") as file:
            script_content = file.read()
        script_content = script_content.replace("FBX_PATH", fbx_path)
        base_urdf_file_name, extension_urdf_file_name = os.path.splitext(urdf_file_name)
        script_content = script_content.replace("ROBOT_NAME", base_urdf_file_name)
        print(script_content)
        base_import_script_path, extension_import_script_path = os.path.splitext(import_script_path)
        new_import_script_path = base_import_script_path + "_new" + extension_import_script_path
        with open(new_import_script_path, "w") as file:
            file.write(script_content)
        command = 'unreal.PythonScriptLibrary.execute_python_command("' + new_import_script_path + '")'
        result = executeCommand(command)
        self.get_logger().info("command end")

    def __del__(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimLancher()
    minimal_publisher.get_logger().info("node start")

    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

