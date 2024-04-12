from setuptools import find_packages, setup

package_name = 'c_megarover_localization'

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
    maintainer='user',
    maintainer_email='yasu.bv0309@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = c_megarover_localization.minimal_publisher:main',
            "global_localization = c_megarover_localization.global_localization:main",
            "transform_fusion = c_megarover_localization.transform_fusion:main",
        ],
    },
)
