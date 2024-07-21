import os


class ConfigParser:
    def __init__(self, directory):
        self.directory = directory

    def parse_files(self):
        parsed_data = {}
        for root, _, files in os.walk(self.directory):
            for file in files:
                if file.endswith('.am'):
                    file_path = os.path.join(root, file)
                    parsed_data[file] = self.parse_file(file_path)
        return parsed_data

    def parse_file(self, file_path):
        with open(file_path, 'r') as file:
            return file.read()  # For now, just read the content. Modify as needed for actual parsing logic.


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Parse configuration files with .am extension.")
    parser.add_argument('directory', type=str, help='Directory path to search for .am files.')

    args = parser.parse_args()

    config_parser = ConfigParser(args.directory)
    parsed_data = config_parser.parse_files()

    for filename, content in parsed_data.items():
        print(f'Parsed content of {filename}:')
        print(content)
        print('---')
