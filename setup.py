from setuptools import find_packages, setup

package_name = 'boxing_2_0_0'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name+'/camera.py']),
        ('lib/' + package_name, [package_name+'/aruco.py']),
        ('lib/' + package_name, [package_name+'/human.py']),
        ('lib/' + package_name, [package_name+'/logPunch.py']),
        ('lib/' + package_name, [package_name+'/punch.py']),
        ('lib/' + package_name, [package_name+'/punchcost.py']),
        ('lib/' + package_name, [package_name+'/coordinateTransformer.py']),
        ('lib/' + package_name, [package_name+'/punchcostfunction.py']),
        ('lib/' + package_name, [package_name+'/point.py']),
        ('lib/' + package_name, [package_name+'/optimalAction.py']),
        ('lib/' + package_name, [package_name+'/graphnode.py']),
        ('lib/' + package_name, [package_name+'/graph.py']),
        ('lib/' + package_name, [package_name+'/controlRotation.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='changsul',
    maintainer_email='changsul@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'main = boxing_2_0_0.main:main',
        	'main_old = boxing_2_0_0.main_old:main',
        	'moveBackHuman = boxing_2_0_0.moveBackHuman:main',
        	'main_grid = boxing_2_0_0.main_grid:main',
        ],
    },
)
