from setuptools import find_packages, setup

package_name = 'ansr_hello_world_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'transformers[torch]',
        'torchvision',
        'torch',
        'Pillow',
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='justin rokisky',
    maintainer_email='justin.rokisky@jhuapl.edu',
    description='A hello world python node for working with the ADK.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = ansr_hello_world_py.publisher:main'
        ],
    },
)
