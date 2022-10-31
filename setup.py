from setuptools import setup

package_name = 'lilhero6_chase_object'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'detect_object = lilhero6_chase_object.detect_object:main',
        	'get_object_range = lilhero6_chase_object.get_object_range:main',
        	'chase_object = lilhero6_chase_object.chase_object:main'
        ],
    },
)
