from setuptools import find_packages, setup

package_name = 'ugv_basic_pubsub'

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
    maintainer='vskskumar',
    maintainer_email='pvskskumar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'talker = ugv_basic_pubsub.talker:main',
        'listener = ugv_basic_pubsub.listener:main',
    ],
},
)
