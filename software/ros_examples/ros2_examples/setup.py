from setuptools import setup
from glob import glob

package_name = 'ros2_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name, glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'hello_ros2 = ros2_examples.hello_ros2:main',
		'simple_pub2 = ros2_examples.simple_pub2:main',
		'simple_sub2 = ros2_examples.simple_sub2:main',
		'simple_srv2 = ros2_examples.simple_srv2:main',
		'simple_cli2 = ros2_examples.simple_cli2:main',
		'simple_action_srv2 = ros2_examples.simple_action_srv2:main',
		'simple_action_cli2 = ros2_examples.simple_action_cli2:main'
        ],
    },
)
