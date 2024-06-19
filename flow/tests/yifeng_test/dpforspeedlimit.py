import json
import os
import subprocess

def modify_params(file_path, update_dict):
    """Load a JSON file, modify it, and save it back."""
    with open(file_path, 'r') as file:
        data = json.load(file)
    data.update(update_dict)
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)

def run_simulation():
    """Run the simulation command."""
    command = "python examples/simulate.py merge --no_render"
    process = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return process.stdout, process.stderr

def main():
    param_updates = [
        {'speed_limit_extra': 10, 'speed_limit_sub' : 30, 'flow_rate' : 1200}, 
        {'speed_limit_extra': 11, 'speed_limit_sub' : 30}, 
        {'speed_limit_extra': 12, 'speed_limit_sub' : 30}
    ]
    json_file_path = 'params.json'

    for update in param_updates:
        # Modify the JSON configuration
        modify_params(json_file_path, update)

        # Run the simulation
        stdout, stderr = run_simulation()

        # Optional: Print/log the output for debugging
        print(f"Running with settings {update}")
        print("Output:", stdout.decode())
        if stderr:
            print("Errors:", stderr.decode())

        # Collect and store data as needed
        # For example, you might want to parse stdout or read generated files

if __name__ == '__main__':
    main()
