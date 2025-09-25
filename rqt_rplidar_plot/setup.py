from setuptools import find_packages, setup

package_name = 'rqt_rplidar_plot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jules',
    maintainer_email='dev@example.com',
    description='An rqt plugin to visualize RPLIDAR laser scan data.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rqt_rplidar_plot = rqt_rplidar_plot.main:main',
        ],
    },
)