import os
import sys
import ctypes

# Load the shared library
lib_paths = ['/home/jelee/anaconda3/envs/DHC/lib/liburdfdom_sensor.so.4.0',
             '/home/jelee/anaconda3/envs/DHC/lib/liburdfdom_model.so.4.0',
             '/home/jelee/anaconda3/envs/DHC/lib/liburdfdom_model_state.so.4.0',
             '/home/jelee/anaconda3/envs/DHC/lib/liburdfdom_world.so.4.0']


for lib_path in lib_paths:
    try:
        lib = ctypes.CDLL(lib_path)
        print(f"Successfully loaded: {lib_path}")
    except OSError as e:
        print(f"Failed to load library: {e}")

# Set library path to ensure Python can find the shared libraries
# os.environ['LD_LIBRARY_PATH'] = '/home/jelee/anaconda3/envs/DHC/lib:' + os.environ.get('LD_LIBRARY_PATH', '')

# # Check if the library can now be imported
# try:
#     import urdfdom
#     print("URDFDOM library loaded successfully!")
# except ImportError as e:
#     print(f"Error: {e}")


sys.path.insert(-1, os.getcwd() + "/bazel-bin/")
sys.path.append(os.getcwd())

# for motion compendium
# print(os.getcwd() + "/dex/trajopt/bazel-bin/")
# sys.path.insert(-1, os.getcwd() + "/dex/trajopt/bazel-bin/")
# sys.path.append(os.getcwd() + "/dex/trajopt")