from setuptools import setup, find_packages

setup(
    name='pathfinding3D',
    version='0.1.0',    
    description='Pathfinding algorithms for 3D voxel environment (based on python-pathfinding)',
    url='https://github.com/harisankar95/python-pathfinding3D',
    author='Harisankar Babu',
    author_email='harisankar995@gmail.com',
    license='MIT',
    packages=find_packages(include=['pathfinding3D']),
    install_requires=['numpy'],
)