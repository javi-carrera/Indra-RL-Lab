import yaml

from use_cases.uc1 import deploy_uc1
from use_cases.uc2 import deploy_uc2
from use_cases.uc3 import deploy_uc3

deploy_uc = {
    'uc1': deploy_uc1,
    'uc2': deploy_uc2,
    'uc3': deploy_uc3
}

def main():
    
    config_file_path = f"config.yml"
    config = yaml.safe_load(open(config_file_path))

    use_case = config['environment']['id']

    deploy_uc.get(use_case)()