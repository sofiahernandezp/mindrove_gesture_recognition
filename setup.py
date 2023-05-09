from setuptools import setup

package_name = 'mindrove_gesture_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sofia',
    maintainer_email='sofia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = mindrove_gesture_recognition.publisher_member_function:main',
            'listener = mindrove_gesture_recognition.subscriber_member_function:main',
        ],
    },
)
