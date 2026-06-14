from setuptools import find_packages, setup

package_name = 'Vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','opencv-python'],
    zip_safe=True,
    maintainer='faisal',
    maintainer_email='mfaisalsaeed18@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'Stream=Vision.Stream:main',
            'Detect_Hurdle=Vision.Hurdle_Detection:main',
            'Detect_Fire=Vision.Fire_Detection:main',
            'Detect_Fight=Vision.Fight_Detection:main',
            'Detect_Weapon=Vision.Weapon_Detection:main',
        ],
    },
)
