from setuptools import find_packages, setup

package_name = 'rl_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add the launch files
        ('share/' + package_name + '/launch', ['launch/test_launch.py']),
        ('share/' + package_name + '/launch', ['launch/train_launch.py']),
        ('share/' + package_name + '/launch', ['launch/deploy_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='114339284+javi-carrera@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test = rl_pkg.test:main",
            "train = rl_pkg.train:main",
            "deploy = rl_pkg.deploy:main",
        ],
    },
)
