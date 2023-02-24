# sound_recognition

This repo constains code for sound classification (like doorbell detection) under ROS 2 architecture using the YAMNet pretrained deep net that predicts 521 audio event classes based on the AudioSet-YouTube corpus (https://github.com/tensorflow/models/tree/master/research/audioset/yamnet).

## Installation

YAMNet depends on the following Python packages:

* [`numpy`](http://www.numpy.org/)
* [`resampy`](http://resampy.readthedocs.io/en/latest/)
* [`tensorflow`](http://www.tensorflow.org/)
* [`pysoundfile`](https://pysoundfile.readthedocs.io/)
* [`librosa`](https://librosa.org/)
* [`pyaudio`](https://people.csail.mit.edu/hubert/pyaudio/)
* [`protobuf`](https://developers.google.com/protocol-buffers/docs/overview?hl=es-419) v.3.20.x

```shell
$ pip3 install numpy resampy tensorflow soundfile librosa pyaudio
$ cd ~/ros2_ws/src
$ git clone git@github.com:MERLIN2-ARCH/sound_recognition.git
$ cd sound_recognition
$ vcs import ~/ros2_ws/src < third_parties.repos
$ cd ~/ros2_ws
$ colcon build
```

## Usage
### Launch

```shell
$ ros2 launch sound_recognition sound_recognition.launch.py
```
### Shell example

```shell
$ ros2 action send_goal /sound_recognition/listen_doorbell sound_recognition_msgs/action/Listen {}
```

## Services

* Service to start the doorbell detection:

service: /sound_recognition/start_ad_listening

type: std_srvs/srv/Empty

```shell
$ ros2 service call /sound_recognition/start_ad_listening std_srvs/srv/Empty
```

* Service to stop the doorbell detection:

service: /sound_recognition/stop_ad_listening

type: std_srvs/srv/Empty

```shell
$ ros2 service call /sound_recognition/stop_ad_listening std_srvs/srv/Empty
```





