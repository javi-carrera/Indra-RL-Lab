from setuptools import find_packages, setup

package_name = 'examples_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='114339284+TheSyntaxERROR98@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_navigation_example = examples_pkg.environments.autonomous_navigation_example_environment:main',
            'shooting_example = examples_pkg.environments.shooting_example_environment:main',
            'train = examples_pkg.train:main',
        ],
    },
)
