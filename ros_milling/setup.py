from setuptools import setup

package_name = 'ros_milling'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        #(os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),    
        #(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        #(os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),           
        #(os.path.join('share', package_name, 'description', 'urdf'), glob(os.path.join('description','urdf', '*.xacro'))),
        #(os.path.join('share', package_name, 'description', 'meshes', 'collision'), glob(os.path.join('description','meshes','collision', '*.stl'))),
        #(os.path.join('share', package_name, 'description', 'meshes', 'visual'), glob(os.path.join('description','meshes','visual', '*.stl'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Claudius Birk',
    maintainer_email='claudiusbirk@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'joint_points_topic = ros_milling.joint_points_topic:main',
                'joint_points_act_service = ros_milling.joint_points_act_service:main',
        ],
    },
)
