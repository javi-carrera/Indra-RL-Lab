import yaml

from use_cases.uc1 import test_uc1
from use_cases.uc2 import test_uc2

test_uc = {
    'uc1': test_uc1,
    'uc2': test_uc2
}

def main():

    config_file_path = f"config.yml"
    config = yaml.safe_load(open(config_file_path))

    use_case = config['environment']['id']

    test_uc.get(use_case)()
    

