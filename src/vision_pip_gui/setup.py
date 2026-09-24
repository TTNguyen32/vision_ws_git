from setuptools import setup

package_name = 'vision_pip_gui'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tin',
    description='rqt control panel for the A*STAR drone vision pipeline',
    license='Proprietary',
)
