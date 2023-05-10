from setuptools import setup

package_name = 'air_nlp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + "/model"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marim694',
    maintainer_email='marioimpesi1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nlp_node = air_nlp.nlp_node:main'
        ],
    },
)
