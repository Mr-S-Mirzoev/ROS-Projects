from setuptools import setup

package_name = 'image_processing_utilities'

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
    maintainer='Sergey Mirzoev',
    maintainer_email='sergey.mirzoev@nami.ru',
    description='Utilities for Image Processing meta-package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slider = image_processing_utilities.slider:main',
            'tf_broadcaster = image_processing_utilities.tf_broadcaster:main'
        ],
    },
)
