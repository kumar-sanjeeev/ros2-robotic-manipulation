from setuptools import find_packages, setup

package_name = 'pkg_python_examples'

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
    maintainer='robotics-noob',
    maintainer_email='kumar.san96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = pkg_python_examples.simple_publisher:main',
            'simple_subscriber = pkg_python_examples.simple_subscriber:main',
            'simple_parameter = pkg_python_examples.simple_parameters:main',
            'simple_service_server = pkg_python_examples.service_add_no:main',
            'simple_service_client = pkg_python_examples.simple_service_client:main'
        ],
    },
)
