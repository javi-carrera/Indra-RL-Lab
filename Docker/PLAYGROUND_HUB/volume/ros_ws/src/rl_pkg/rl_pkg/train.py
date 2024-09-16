import os
import yaml

from use_cases.uc1 import train_uc1
from use_cases.uc2 import train_uc2

def main():
    
    username = os.environ.get('USERNAME')

    config_file_path = f"/home/{username}/config.yml"

    config = yaml.safe_load(open(config_file_path))

    use_case = config['use_case']


    match use_case:
        case 'uc1':
            print("Training UC1")
            train_uc1()
        case 'uc2':
            print("Training UC2")
            train_uc2()
        case _:
            print(f"Invalid use case: {use_case}")
            raise ValueError(f"Invalid use case: {use_case}")