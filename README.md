# YAMNet_ros2

This repo constains code for sound classification (like doorbell detection) under ROS 2 (Galactic) architecture using the YAMNet pretrained deep net that predicts 521 audio event classes based on the AudioSet-YouTube corpus (https://github.com/tensorflow/models/tree/master/research/audioset/yamnet).

## Installation

YAMNet depends on the following Python packages:

* [`numpy`](http://www.numpy.org/)
* [`resampy`](http://resampy.readthedocs.io/en/latest/)
* [`tensorflow`](http://www.tensorflow.org/)
* [`pysoundfile`](https://pysoundfile.readthedocs.io/)

```shell
$ pip3 install numpy resampy tensorflow soundfile
$ cd ~/ros2_ws/src
$ git clone git@github.com:uleroboticsgroup/simple_node.git
$ git clone git@github.com:igonzf06/yamnet_ros2.git
$ cd ~/ros2_ws
$ colcon build
```

## Usage
### Launch

```shell
$ ros2 launch audio_detection audio_detection.launch.py
```
### Shell example

```shell
$ ros2 action send_goal /audio_detection/listen_doorbell audio_detection_interfaces/action/Listen {}
```

## Services

* Service to start the doorbell detection:

service: /audio_detection/start_ad_listening

type: std_srvs/srv/Empty

```shell
$ ros2 service call /audio_detection/start_ad_listening std_srvs/srv/Empty
```

* Service to stop the doorbell detection:

service: /audio_detection/stop_ad_listening

type: std_srvs/srv/Empty

```shell
$ ros2 service call /audio_detection/stop_ad_listening std_srvs/srv/Empty
```





