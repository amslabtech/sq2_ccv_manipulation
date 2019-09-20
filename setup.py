from setuptools import setup

package_name = 'sq2_ccv_manipulation'

setup(
    name=package_name,
    version='0.6.1',
    packages=[],
    py_modules=[
        'spawn_ccv',
        'twist_ccv',
        'keyop_ccv',
        'collision_detection'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='yourname',
    author_email='pritish.debasis96@gmail.com',
    maintainer='yourname',
    maintainer_email='email@hoge',
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
            'spawn_ccv = spawn_ccv:main',
            'twist_ccv = twist_ccv:main',
            'keyop_ccv = keyop_ccv:main',
            'collision_detection = collision_detection:main'
            ],
    },
)
