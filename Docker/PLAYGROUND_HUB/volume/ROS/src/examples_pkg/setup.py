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
            'uc1_v0_test = examples_pkg.uc1_v0.test:test',
            'uc1_v0_train = examples_pkg.uc1_v0.train:train',
            # 'uc2_v0_test = examples_pkg.uc1_v0.test:test',
            'uc2_v0_train = examples_pkg.uc2_v0.train:train',
        ],
    },
)
