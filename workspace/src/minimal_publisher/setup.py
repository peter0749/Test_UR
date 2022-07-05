from setuptools import setup

package_name = 'py_pub'

setup(
    name=package_name,
    version='0.16.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Aditya Pande, Shane Loretz',
    maintainer_email='aditya.pande@openrobotics.org, shane@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal publishers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pub.talker:main',
            'publisher_old_school = py_pub.publisher_old_school:main',
            'publisher_local_function = py_pub.publisher_local_function:main',
            'publisher_member_function = py_pub.publisher_member_function:main',
        ],
    },
)
