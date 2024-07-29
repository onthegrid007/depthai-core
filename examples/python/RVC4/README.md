# Getting started on RVC4

This folder contains some examples to quickly get started on an RVC4 device.

There are two modes of running the examples.

### Running on a host PC (default)
In this case install depthai library on your host PC and run the examples as normal.
To do this:

``` bash
git clone https://github.com/luxonis/depthai-core.git
git checkout v3_develop
python3 depthai-core/examples/python/install_requirements.py
cd depthai-core/examples/python/RVC4
python3 Camera/camera_output.py # Use any of the examples in the RVC4 folder
```


### Running on device
For running the examples on device you can first ssh into the device and then run the examples, where you have to remove any visualizing parts.

For the simple `camera_output.py` example you have to do the following:

``` bash
git clone https://github.com/luxonis/depthai-core.git
git checkout v3_develop
# Copy the depthai-core folder to the device
scp -r depthai-core root@IP_ADDRESS
ssh root@IP_ADDRESS
python3 -m ensurepip
python3 depthai-core/examples/python/install_requirements.py

# Remove the `cv2.imshow` part from the example
cd depthai-core/examples/python/RVC4
python3 Camera/camera_output.py
```