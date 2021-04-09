import sys

# see 
# https://github.com/scikit-build/scikit-build-sample-projects/blob/master/projects/hello-pybind11/setup.py
try:
    from skbuild import setup
except ImportError:
    raise Exception

setup(
    name="voxblox_ros_python",
    version="0.0.1",
    description="python interface to voxblox client",
    author='Hirokazu Ishida',
    license="MIT",
    packages=["voxblox_ros_python"],
    package_dir={'': 'python'},
    cmake_install_dir='python/voxblox_ros_python/'
    )
