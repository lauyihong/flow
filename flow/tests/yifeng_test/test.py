
# param_updates = [
#     {'speed_limit_extra': 10, 'speed_limit_sub' : 30, 'flow_rate' : 1200}, 
#     # {'speed_limit_extra': 11, 'speed_limit_sub' : 30}, 
#     # {'speed_limit_extra': 12, 'speed_limit_sub' : 30}
# ]
import json
import os
import subprocess
from tqdm import tqdm

def modify_params(file_path, update_dict):
    """Load a JSON file, modify it, and save it back."""
    with open(file_path, 'r') as file:
        data = json.load(file)
    data.update(update_dict)
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=4)

def run_simulation():
    """Run the simulation command."""
    command = "python ../../examples/simulate.py merge --no_render"
    process = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return process.stdout, process.stderr

json_file_path = 'params.json'

if __name__ == '__main__':
    param_updates = []

    # Adding fixed values of speed_limit_sub
    for main_speed in [25,30]:
        for sub_speed in [10,15,20,25,30]:
            if main_speed > sub_speed:
                # Loop for speed_limit_extra from 10 to 30, inclusive, increment by 1
                for extra_speed in range(10, 31):
                    # Loop for flow_rate from 900 to 1300, inclusive, increment by 100
                    for flow_rate in range(900, 1301, 100):
                        param_updates.append({
                            'speed_limit_main': main_speed,
                            'speed_limit_extra': extra_speed,
                            'speed_limit_sub': sub_speed,
                            'flow_rate': flow_rate
                        })

    # # Printing or using the param_updates list
    # for params in param_updates:
    #     print(params)

    for update in tqdm(param_updates):
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
