# import argparse
# import subprocess
# import sys
# import os
# from Parser.parser import ParserWrapper
#
# class DirectoryParser:
#     def __init__(self, directory):
#         self.directory = directory
#
#     def parse_files(self):
#         parsed_data = {}
#         for root, _, files in os.walk(self.directory):
#             for file in files:
#                 if file.endswith('.am'):
#                     file_path = os.path.join(root, file)
#                     with open(file_path, 'r') as file:
#                         content = file.read()
#                         base_name = os.path.basename(file_path)
#                         file_name = os.path.splitext(base_name)[0]
#                         parser = ParserWrapper(file_name=file_name, content=content)
#                         parsed_data[file_name] = parser.generate_python_code()
#
#                     # parsed_data[file] = self.parse_file(file_path)
#         return parsed_data
#
#     def parse_file(self, file_path):
#         with open(file_path, 'r') as file:
#             return file.read()  # For now, just read the content. Modify as needed for actual parsing logic.
#
#
# def main(args):
#     middleware_dir = args.middleware_dir
#     ros_workspace = args.ros_workspace
#     ros_nodes = args.ros_nodes
#
#     # Print the received arguments
#     print(f'Middleware Directory: {middleware_dir}')
#     print(f'ROS Workspace: {ros_workspace}')
#     print(f'ROS2 Nodes to run: {ros_nodes}')
#
#     parsed_data = DirectoryParser(middleware_dir).parse_files()
#     a=1
# if __name__ == '__main__':
#     parser = argparse.ArgumentParser(
#         description='')
#     parser.add_argument('--middleware-dir', type=str, required=True, help='Path to the middleware files directory.')
#     parser.add_argument('--ros-workspace', type=str, required=True, help='Path to the ROS2 workspace.')
#     parser.add_argument('--ros-nodes', type=str, nargs='+', required=True, help='Array of ROS2 nodes to execute alongside execute.')
#
#     args = parser.parse_args()
#     main(args)