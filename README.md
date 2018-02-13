# Maya
This repo provides some example scripts to launch maya simulator, read vehicle data, LIDAR point clouds,
images, HD map data, as well as send control commands to the vehicles in maya.

Author: Dr. Z. Xing\
Date: Feb 2018

## setup

- brew install geos
- conda env create -f environment.yml
- source activate maya_public
- pip uninstall nanomsg
- pip install --global-option=build_ext --global-option="-I/usr/local/include/nanomsg/" nanomsg

# compile the protocol buffer
``python -m grpc_tools.protoc -I./protos/ --python_out=. --grpc_python_out=. ./protos/message.proto``
