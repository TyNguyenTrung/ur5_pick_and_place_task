from setuptools import setup

package_name = 'camera_viewer'

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
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='RGB camera subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_test_1 = camera_viewer.py_test_1:main',
            'py_test_2 = camera_viewer.rgb_subscriber:main',
        ],
    },
)

