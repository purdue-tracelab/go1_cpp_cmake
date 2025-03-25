from setuptools import setup, find_packages

package_name = 'go1_cpp_cmake'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools', 'matplotlib', 'pandas'],  # Add any Python dependencies here
    entry_points={
        'console_scripts': [
            'plot_data = go1_cpp_cmake.plot_data:main',  # Add your Python script entry point
        ],
    },
)