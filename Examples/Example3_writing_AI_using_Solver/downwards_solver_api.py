import subprocess
import sys
import re
import json
import os
class DownwardsAPI:
    def __init__(self, domain_file, problem_file):
        self.domain_file = domain_file
        self.problem_file = problem_file
        # Get the path of the current executable
        executable_path = os.path.realpath(sys.executable)
        executable_path = os.path.realpath(__file__)
        # Get the directory of the current executable
        self.cwd = os.path.dirname(executable_path)
        a=1

    def generate_plan(self):
        output = self.run_planner()
        actions = self.parse_plan(output)

        #print plan
        dict_as_string = json.dumps(actions)
        print(dict_as_string)

    def run_planner(self):
        # Define the command to run the planner
        planner_path = os.path.join(self.cwd, 'fast-downward.py')
        domain_path = os.path.join(self.cwd, '..', 'pddl', self.domain_file)
        problem_path = os.path.join(self.cwd, '..', 'pddl', self.problem_file)

        command = [
            "python3",
            planner_path,
            domain_path,
            problem_path,
            "--search",
            "astar(lmcut())"
        ]

        # # Print the command for debugging purposes
        # print("Running command:", " ".join(command))

        # Run the planner
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        if result.returncode != 0:
            print("Planner stderr output:\n", result.stderr)
            raise RuntimeError(f"Planner failed with error code {result.returncode}:\n{result.stderr}")

        # Print the planner stdout for debugging purposes
        # print("Planner stdout output:\n", result.stdout)

        return result.stdout


    def parse_plan(self, plan_output):
        actions = []
        needed_lines = []
        capture = False

        for line in plan_output.splitlines():
            if "Actual search time" in line:
                capture = True
                continue
            if "Plan length" in line:
                break
            if capture:
                needed_lines.append(line)

        for l in needed_lines:
            actions.append(l.split(' ')[:-1])

        return actions


def main(domain_file = 'domain.pddl', problem_file='problem.pddl'):
    down_api = DownwardsAPI(domain_file, problem_file)
    down_api.generate_plan()
    # plan_output = run_planner(domain_path, problem_path, planner_path, search_algorithm)
    # actions = parse_plan(plan_output)
    # return actions


if __name__ == "__main__":
    main()
    exit()
    # if len(sys.argv) != 3:
    #     print(f" arg_count:{len(sys.argv)} Usage: python3 downwards_solver_api.py <domain_path> <problem_path>")
    #     sys.exit(1)
    # domain_path = sys.argv[1]
    # problem_path = sys.argv[2]
    # search_algorithm = '' #sys.argv[3]
    # planner_path = '/home/or/Downloads/downward/fast-downward.py'
    # actions = main(domain_path, problem_path, planner_path, search_algorithm)
    # dict_as_string = json.dumps(actions)
    # print(dict_as_string)
