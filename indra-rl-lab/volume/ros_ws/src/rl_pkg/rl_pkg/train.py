import yaml

from use_cases.uc1 import train_uc1
from use_cases.uc2 import train_uc2
from use_cases.uc3 import train_uc3

train_uc = {
    'uc1': train_uc1,
    'uc2': train_uc2,
    'uc3': train_uc3
}

def main():

    config_file_path = f"config.yml"
    config = yaml.safe_load(open(config_file_path))

    use_case = config['environment']['id']

    train_uc.get(use_case)()