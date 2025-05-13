from setuptools import find_packages, setup

package_name = 'inrof'

setup(
    name=package_name,
    version='0.0.1',  # Updated version
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/run_simple_robot.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hak',
    maintainer_email='yamada.hayato.t6@dc.tohoku.ac.jp',
    description='Simple diff-drive robot package',  # Updated description
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_drive = inrof.inrof_auto_drive:main',
        ],
    },
)
