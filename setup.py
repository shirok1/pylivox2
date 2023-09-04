from setuptools import find_packages, setup

package_name = 'livox2_ctrl'

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
    maintainer='Shiroki Satsuki',
    maintainer_email='me@shirok1.dev',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_pub = livox2_ctrl.publisher:main',
            'daemon = livox2_ctrl.daemon:main',
        ],
    },
)
