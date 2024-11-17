import os
import glob
import signal
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory
from os.path import expanduser

def find_unreal_engine_executable(base_dir="~/UnrealEngine"):
    # Expand user directory in the base path
    base_dir = os.path.expanduser(base_dir)
    
    # Search for the Unreal Engine executable within any version folder
    unreal_engine_executable_pattern = os.path.join(base_dir, "*/*/Linux/UnrealEditor")
    unreal_engine_executables = glob.glob(unreal_engine_executable_pattern)
    
    if unreal_engine_executables:
        print("Unreal Engine executable(s) found:")
        for executable in unreal_engine_executables:
            print(executable)
        return str(unreal_engine_executables[0])
    else:
        print("No Unreal Engine executable found.")
        return None

class SimLancher(Node):
    def __init__(self):
        super().__init__('sim_launcher')
        self.proc = None

        self.declare_parameter('unreal_engine_path', '')
        unreal_engine_path = self.get_parameter('unreal_engine_path').get_parameter_value().string_value
        found_unreal_engine_path = find_unreal_engine_executable()
        if (unreal_engine_path == '') and (not found_unreal_engine_path == None):
            unreal_engine_path = found_unreal_engine_path
        if not os.path.isfile(unreal_engine_path):
            self.get_logger().fatal('Unreal Engine not found!!')
            return

        self.declare_parameter('project_path', str(os.path.expanduser("~")) + '/work/RobotUE5App/RobotUE5App.uproject')
        project_path = self.get_parameter('project_path').get_parameter_value().string_value

        self.declare_parameter('open_file_path', '')
        open_file_path = self.get_parameter('open_file_path').get_parameter_value().string_value
                
        command = [unreal_engine_path, project_path]
        if not open_file_path == '':
            command.append('-openfile')
            command.append(open_file_path)
        print(command)
        self.proc = subprocess.Popen(command, preexec_fn=os.setsid)        
    
    def __del__(self):
        if not self.proc == None:
            if self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimLancher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

