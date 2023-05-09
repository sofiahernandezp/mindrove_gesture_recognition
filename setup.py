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
            'dataset_publisher = mindrove_gesture_recognition.csv_dataset_publisher:main',
            'gesture_classfier = mindrove_gesture_recognition.gesture_classifier:main',
        ],
    },
)
