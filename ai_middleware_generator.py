import argparse
import os
import shutil
import subprocess
import sys

from Parser.parser import ParserWrapper


class PackageUtils:
    def __init__(self, package_name, workspace_dir):
        self.workspace_dir = workspace_dir
        self.package_name = package_name
        self.maintainer_email = 'orwert@post.bgu.ac.il'
        self.maintainer_name = 'Or'
    def remove_existing_package(self, package_name, src_dir):
        package_dir = os.path.join(src_dir, package_name)
        if os.path.exists(package_dir):
            print(f"Removing existing package directory: {package_dir}")
            shutil.rmtree(package_dir)
            print(f"Package {package_name} removed successfully.")
        else:
            print(f"No existing package named {package_name} found.")

    def get_package_xml_file_content(self):
        content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{self.package_name}</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="{self.maintainer_email}">{self.maintainer_name}</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <build_depend>rclpy</build_depend>
  <build_depend>redis</build_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>redis</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""
        return content
    def get_launch_file_content(self, external_nodes, manager_nodes):
        content = """import sys
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(["""

        for node_name in external_nodes:
            content += f"""
        Node(
        package='{external_nodes[node_name]}',
        executable='{node_name}',
        name='{node_name}1',
        output='screen',
        parameters=[{{'param_name': 'param_value'}}]
    ),"""
        for node_name in manager_nodes:
            content += f"""
        Node(
        package='{self.package_name}',
        executable='{node_name}',
        name='{node_name}1',
        output='screen',
        parameters=[{{'param_name': 'param_value'}}]
    ),
    """
        content += """
    ])
if __name__ == '__main__':
    from launch import LaunchService
    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(generate_launch_description())
    sys.exit(launch_service.run())
        """
        return content

    def get_setup_file_content(self, manager_nodes):
        content = f"""from setuptools import setup
import os
from glob import glob
package_name = '{self.package_name}'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='{self.maintainer_name}',
    maintainer_email='{self.maintainer_email}',
    description='TODO: Package description',
    license='Apache License 2.0', 
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': ["""
        for manager_node in manager_nodes:
            content +=f"""
         '{manager_node}= {self.package_name}.{manager_node}:main',"""
        content +="""
        ],
    },
)
"""
        return content

    def create_ros2_python_package(self, external_nodes={}, manager_nodes ={}, dont_run = False):
        self.dont_run = dont_run
        # Define the workspace and package paths
        workspace_dir = os.path.expanduser(self.workspace_dir)
        src_dir = os.path.join(workspace_dir, 'src')
        package_dir = os.path.join(src_dir, self.package_name)
        launch_dir = os.path.join(package_dir, 'launch')
        resource_dir = os.path.join(package_dir, 'resource')
        package_py_dir = os.path.join(package_dir, self.package_name)

        # Remove any existing package with the same name
        self.remove_existing_package(self.package_name, src_dir)

        # Create the package using ros2 command
        subprocess.run(['ros2', 'pkg', 'create', '--build-type', 'ament_python', self.package_name], cwd=src_dir)

        # Create additional directories
        os.makedirs(launch_dir, exist_ok=True)
        os.makedirs(resource_dir, exist_ok=True)
        os.makedirs(package_py_dir, exist_ok=True)

        # Create a sample launch file
        launch_file_content = self.get_launch_file_content(external_nodes, manager_nodes)

        with open(os.path.join(launch_dir, f'{self.package_name}.launch.py'), 'w') as f:
            f.write(launch_file_content)

        # Create a sample node file
        for manager_name in manager_nodes:
            node_file_content = manager_nodes[manager_name]
            with open(os.path.join(package_py_dir, f'{manager_name}.py'), 'w') as f:
                f.write(node_file_content)

        #Create other python files
        python_files = self.get_additional_python_files()
        for file in python_files:
            with open(os.path.join(package_py_dir, file), 'w') as f:
                f.write(python_files[file])

        # Modify setup.py
        setup_file = os.path.join(package_dir, 'setup.py')
        setup_content = self.get_setup_file_content(manager_nodes)
        with open(setup_file, 'w') as f:
            f.write(setup_content)

        package_file = os.path.join(package_dir, 'package.xml')
        package_content = self.get_package_xml_file_content()
        with open(package_file, 'w') as f:
            f.write(package_content)

        # Build the package
        subprocess.run(['colcon', 'build', '--packages-select', f'{self.package_name}'], cwd=workspace_dir)

        # Create a shell script to source the environment and run the package
        run_script_content = f"""\#!/bin/bash
source {workspace_dir}/install/setup.bash 
ros2 launch {self.package_name} {self.package_name}.launch.py
    """

        run_script_path = os.path.join(workspace_dir, 'run_package.sh')
        with open(run_script_path, 'w') as f:
            f.write(run_script_content)

        # Make the run script executable
        os.chmod(run_script_path, 0o755)
        try:
            if not self.dont_run:
                subprocess.run(['/bin/bash', run_script_path], check=False, text=False, capture_output=False)
            #print("Script output:", result.stdout)
        except subprocess.CalledProcessError as e:
            print("Script failed with error code:", e.returncode)
            print("Error output:", e.stderr)
            print("Standard output:", e.stdout)

        print(f"Package {self.package_name} created and built successfully.")
        print(f"To run the package, execute the following script:")
        print(f"{run_script_path}")

    def get_additional_python_files(self):
        files = {}
        files["Utils.py"] = """
import traceback
import redis
class Utils:
    is_init = False
    r = None
    @classmethod
    def init_utils(cls):
        if cls.is_init:
            return
        cls.is_init = True
        cls.r = redis.Redis(host='localhost', port=6379)
    @staticmethod
    def get_trace():
        return traceback.extract_stack()

    @classmethod
    def skill_message(cls, skill_name, verb, parameters):
        print (f'skill_name:{skill_name}, verb:{verb}, parameters:{parameters}')
        hash_key = skill_name + ":"
        with cls.r.pipeline() as pipe:
            pipe.hset(hash_key, verb, 1)
            for field, value in parameters.items():
                if isinstance(value, bool):
                    value = int(value)
                pipe.hset(hash_key, field, value)
            # Execute all commands in the pipeline
            pipe.execute()
        """
        files["SkillManagerBase.py"] = f"""
import rclpy
import threading
from rclpy.node import Node
import redis
from enum import Enum

from {self.package_name}.Utils import Utils


class SkillStateEnum(Enum):
    Waiting = 1
    Monitoring = 2
    Running = 3

class SkillManagerType(Enum):
    Background = 1,
    Reactive = 2


class SkillManagerBase(Node):
    def __init__(self, skill_name, manager_id, skill_manager_type):
        super().__init__('manager_' + skill_name+ manager_id)
        self.monitor_init = False
        self.dont_monitor = True
        Utils.init_utils()
        self.skill_manager_type = skill_manager_type
        self.skill_state = SkillStateEnum.Waiting
        self.START_SKILL_ACTION = 'start_skill'
        self.TERMINATE_SKILL_ACTION = 'terminate_skill'
        self.parameters = None
        self.persistent = None
        self.termination_modes = None
        self.volatiles = None
        self.skill_name = skill_name
        self.skill_name_unique = 'manager_' + skill_name+ manager_id
        self.topic_subscriptions = {{}}

        self.r = redis.Redis(host='localhost', port=6379)

        self.redis_listener_thread = threading.Thread(target=self.redis_listener, daemon=True)
        self.redis_listener_thread.start()

        if skill_manager_type == SkillManagerType.Background:
            self.start_monitoring()


    def enable_keyspace_notifications(self):
        # Enable notifications for all key events related to hash sets and expirations
        self.r.config_set('notify-keyspace-events', 'Kh')
        #print("Enabled keyspace notifications.")
    def destroy_node(self):
        self.get_logger().info('Cleaning up before shutting down the node (data_volatile clean).')
        super().destroy_node()  # Call the parent class destroy_node method
        if self.volatiles is not None:
            self.volatiles.cleanup()
        if self.parameters is not None:
            self.parameters.cleanup()
        if self.termination_modes is not None:
            self.termination_modes.cleanup()

        self.termination_modes.cleanup()

    def handle_redis_message_to_skill(self, command, skill_parameters):
        if command == "RUN":
            self.get_logger().info(f'command == "RUN", skill manager state:{{self.skill_state}}')
            if self.skill_state == SkillStateEnum.Waiting:
                self.start_monitoring()
                self.start_execution(skill_parameters)
            if self.skill_state == SkillStateEnum.Monitoring:
                self.start_execution(skill_parameters)

        elif command == "MONITOR":
            if self.skill_state == SkillStateEnum.Waiting:
                self.start_monitoring(skill_parameters)
            if self.skill_state == SkillStateEnum.Running:
                self.stop_execution()
            self.sksill_state = SkillStateEnum.Monitoring
        elif command == "WAIT":
            if self.skill_state == SkillStateEnum.Monitoring:
                self.stop_monitoring()
            if self.skill_state == SkillStateEnum.Running:
                self.stop_execution()
                self.stop_monitoring()
        else:
            self.get_logger().error(f"Skill '{{self.skill_name}}' listener, Command not recognized: {{command}}")
    def added_skill_callback(self, message):
        self.get_logger().info(f'Recieved a redis message for:{{self.skill_name}}')

        key = 'not set'
        try:
            key = message['channel'].decode('utf-8')
            if message['data'] == b'del':
                self.get_logger().info(f'{{key}} was deleted')
                return
            ind = key.find(self.skill_name)
            if ind >= 0:
                key = key[ind:]
            else:
                return

            # Fetch the hash set associated with the key
            skill_data = self.r.hgetall(key)

            #Delete redis key so that the same request can be sent again
            if self.r.exists(key):
                # Delete the key
                result = self.r.delete(key)
                if result == 1:
                    print(f"redis request using key:'{{key}}' was deleted.")

            self.get_logger().info(f'skill data: {{skill_data}}')
            self.get_logger().info(f'skill data end')
            if skill_data:
                skill_parameters ={{k.decode('utf-8'): v.decode('utf-8') for i, (k, v) in enumerate(skill_data.items()) if i != 0}}
                command = next(iter(skill_data)).decode('utf-8')
                self.get_logger().info(f"command:{{command}} parameters: {{skill_parameters}}")
                self.handle_redis_message_to_skill(command, skill_parameters)
        except Exception as e:
            self.get_logger().error(f"Error processing key '{{key}}': {{e}}, trace:{{Utils.get_trace()}}")



    def redis_listener(self):
        self.enable_keyspace_notifications()
        pubsub = self.r.pubsub()
        pattern = f'__keyspace@0__:{{self.skill_name}}:*'
        pubsub.psubscribe(**{{pattern: self.added_skill_callback}})
        #print(f"Subscribed to pattern '{{pattern}}'")

        self.get_logger().info(f"Listening for redis keyspace events for skill: '{{self.skill_name}}:' ")
        try:
            for message in pubsub.listen():
                if message['type'] == 'pmessage':
                    self.added_skill_callback(message)
        except redis.exceptions.ConnectionError as e:
            self.get_logger().error(f"Redis connection error: {{e}}")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {{e}}")


    def start_monitoring(self, parameters=None):
        self.skill_state = SkillStateEnum.Monitoring
        self.dont_monitor = False
        self.get_logger().info(
            f"Start monitoring skill: {{self.skill_name}}")

    def stop_monitoring(self, parameters=None):
        self.dont_monitor = True
        self.skill_state = SkillStateEnum.Waiting
        self.get_logger().info(
            f"Stop monitoring skill: {{self.skill_name}}")


    def start_execution(self, parameters=None):
        self.get_logger().error(
            f"Try to start executing skill: {{self.skill_name}}, the procedure was not defined")

    def stop_execution(self):
        if self.skill_manager_type == SkillManagerType.Background:
            self.skill_state = SkillStateEnum.Monitoring
        if self.skill_manager_type == SkillManagerType.Reactive:
            self.stop_monitoring()
        
        """
        files["SharedData.py"] = f"""
import redis

from {self.package_name} import SkillManagerBase
from enum import Enum

class VariableType(Enum):
    PARAMETERS = 1
    PERSISTENT = 2
    EXTERNAL_STATE = 3,
    TERMINATION_MODE = 4,
    VOLATILE = 5,

class SharedData:
    def __init__(self, manager_node : SkillManagerBase, var_type : VariableType):
        self.manager_node = manager_node
        self.is_volatile = var_type in [VariableType.PARAMETERS, VariableType.VOLATILE, VariableType.TERMINATION_MODE]
        self.prefix = ''
        if var_type == VariableType.PARAMETERS:
            self.prefix = manager_node.skill_name_unique + "_PARAMETERS_"
        if var_type == VariableType.VOLATILE:
            self.prefix = manager_node.skill_name_unique + "_VOLATILE_"
        if var_type == VariableType.TERMINATION_MODE:
            self.prefix = manager_node.skill_name_unique + "_TERMINATION_MODE_"
        if var_type == VariableType.PERSISTENT:
            self.prefix = "PERSISTENT_"
        self.r = redis.Redis(host='localhost', port=6379, decode_responses=True)

    def cleanup(self):
        if self.is_volatile:
            # Use SCAN to find keys
            cursor = '0'
            keys_to_delete = []
            while cursor != 0:
                cursor, keys = self.r.scan(cursor=cursor, match=f'{{self.prefix}}*')
                keys_to_delete.extend(keys)
"""
        return files
class DirectoryParser:
    def __init__(self, am_files_directory):
        self.directory = am_files_directory

    def parse_files(self, package_name):
        parsed_data = {}
        for root, _, files in os.walk(self.directory):
            for file in files:
                if file.endswith('.am'):
                    file_path = os.path.join(root, file)
                    with open(file_path, 'r') as file:
                        content = file.read()
                        base_name = os.path.basename(file_path)
                        file_name = os.path.splitext(base_name)[0]
                        parser = ParserWrapper(file_name=file_name, content=content, package_name=package_name)
                        parsed_data[file_name] = parser.get_python_code()

                    # parsed_data[file] = self.parse_file(file_path)
        return parsed_data






def main():
    package_name = 'ai_middleware_auto_generated'
    debug = True
    parser = argparse.ArgumentParser(description="Run AI-Middleware with specified nodes.")
    parser.add_argument('ros2_workspace', help='Path to the ROS 2 workspace')
    parser.add_argument('am_files_directory', help='Path to the AM files directory')
    parser.add_argument('nodes', nargs='+', help='List of nodes to run in the format package_name/node')

    args = parser.parse_args()

    # Create the node dictionary
    node_dict = {}

    dont_run = False
    if debug:
        dont_run=True
        am_files_directory='/home/or/Projects/AI-Middleware-ROS2/Examples/Example2_writing_AI'
        workspace_dir = '~/ros2_ws'
        node_dict = {'turtlesim_node': 'turtlesim'}
    else:
        am_files_directory = args.am_files_directory
        workspace_dir = args.ros2_workspace
        for node in args.nodes:
            package_name, node_name = node.split('/')
            if package_name not in node_dict:
                node_dict[package_name] = []
            node_dict[package_name].append(node_name)

    dp = DirectoryParser(am_files_directory=am_files_directory)
    parsed_nodes = dp.parse_files(package_name=package_name)
    pkg = PackageUtils(package_name=package_name, workspace_dir=workspace_dir)

    # manager_nodes = ['turn_manager', 'set_pen_manager']
    pkg.create_ros2_python_package(external_nodes=node_dict, manager_nodes=parsed_nodes, dont_run = dont_run)

if __name__ == '__main__':
    main()