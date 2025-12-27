from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='karthik',
    maintainer_email='karthik042412@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test_node_a = my_py_pkg.my_test_node:test_node_a",
            "test_node_b = my_py_pkg.my_test_node:test_node_b",
            "test_node_c = my_py_pkg.my_test_node:test_node_c",
        ],
    },
)
