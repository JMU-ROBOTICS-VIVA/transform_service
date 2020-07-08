from setuptools import setup

package_name = 'transform_service_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/',
         ['launch/transform_service_demo_launch.py']),
        ('share/' + package_name + '/rviz/',
         ['rviz/overhead.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spragunr',
    maintainer_email='nathan.r.sprague@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transform_tester = transform_service_demo.transform_tester:main'
        ],
    },
)
