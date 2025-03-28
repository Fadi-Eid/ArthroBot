from setuptools import find_packages, setup

package_name = 'arthrobot_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'arthrobot_client.arthrobot_servo_client',
        'arthrobot_client.arthrobot_control_joystick',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fadieid',
    maintainer_email='fadieid.eng@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arthrobot_servo_client = arthrobot_client.arthrobot_servo_client:main',
            'arthrobot_control_joystick = arthrobot_client.arthrobot_control_joystick:main'
        ],
    },
)
