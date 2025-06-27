from setuptools import setup, find_packages

package_name = 'color_path_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),   # ← 디렉토리명과 같아야 해
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yunjung',
    maintainer_email='yunjung@example.com',
    description='Color-based tracking demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'color_tracker = color_path_tracker.color_tracker:main',
        ],
    },
)

