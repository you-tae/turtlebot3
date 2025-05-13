from setuptools import find_packages, setup

package_name = 'turtlebot3_cloud_transformer'

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
    maintainer='dbxotjs',
    maintainer_email='dbxotjs1234@khu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cloud_transformer = turtlebot3_cloud_transformer.cloud_transformer:main'
        ],
    },
)
