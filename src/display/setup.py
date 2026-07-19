from setuptools import find_packages, setup


package_name = 'display'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Robot result display',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'display = display.node:main',
        ],
    },
)
