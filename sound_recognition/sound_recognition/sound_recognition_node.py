
import time
import pyaudio
import librosa
import re
import numpy as np

from ament_index_python.packages import get_package_share_directory

import sound_recognition.params as params
import sound_recognition.yamnet as yamnet_model

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Empty
from sound_recognition.custom_thread import CustomThread

class_acepted = ['Doorbell', 'Bell', 'Ding-dong',
                 'Tubular bells', 'Reversing beeps', 'Beep, bleep']


class SoundRecongnitionNode(Node):

    def __init__(self) -> None:

        super().__init__("sound_recognition_node")

        self.__listen_thread = None
        self._model = yamnet_model.yamnet_frames_model(params)
        self._model.load_weights(get_package_share_directory(
            'sound_recognition') + '/yamnet.h5')
        self._classes = yamnet_model.class_names(
            get_package_share_directory('sound_recognition') + '/yamnet_class_map.csv')

        # started_param_name = "started"
        self.started = True
        # self.declare_parameter(started_param_name, True)

        # self.started = self.get_parameter(
        # started_param_name).get_parameter_value().bool_value

        # ring publisher
        self.__pub = self.create_publisher(String, "sound_recognition", 10)

        # service servers
        self.__start_server = self.create_service(
            Empty, "start_ad_listening", self.__start_ad_srv)
        self.__stop_server = self.create_service(
            Empty, "stop_ad_listening", self.__stop_ad_srv)

        if self.started:
            self._start_ad()

    def label_parser(self, label) -> str:
        new_label = re.sub('\s+|,\s+', '_', label.lower())
        self.get_logger().info(new_label)
        return new_label

    # LISTEN
    def listen_from_mic(self) -> None:
        """ listen from mic """
        self._p = pyaudio.PyAudio()
        frame_len = int(params.SAMPLE_RATE * 1)  # 1sec
        self._stream = self._p.open(format=pyaudio.paInt16,
                                    channels=1,
                                    rate=params.SAMPLE_RATE,
                                    input=True,
                                    frames_per_buffer=frame_len)

        while self.started and rclpy.ok():
            # self.get_logger().info("Listening...")

            # data read from mic
            data = self._stream.read(frame_len, exception_on_overflow=False)

            # convert byte to float
            frame_data = librosa.util.buf_to_float(
                data, n_bytes=2, dtype=np.int16)

            # model prediction
            scores, melspec = self._model.predict(
                np.reshape(frame_data, [1, -1]), steps=1)
            prediction = np.mean(scores, axis=0)
            predictions = np.argsort(prediction)[::-1]

            top1 = String()
            top1.data = self.label_parser(self._classes[predictions[0]])
            # self.get_logger().info("... a "+top1.data)

            # if is a doorbell
            # if top1.data in class_acepted:
            # self.__pub.publish(top1)
            # self._stop_ad()

            self.__pub.publish(top1)

    def __listen_thread_cb(self) -> None:
        """ thread callback to listen """

        try:
            self.get_logger().info("listen_thread starts listening")
            self.listen_from_mic()
        finally:
            self.get_logger().info("listen_thread ends")

    # START
    def __start_ad_srv(
        self,
        req: Empty.Request,
        res: Empty.Response
    ) -> Empty.Response:
        """ service to start listen

        Args:
            req(Empty.Request): empty
            res(Empty.Response): empty

        Returns:
            Empty.Response: empty
        """

        self.start_ad()
        return res

    def start_ad(self) -> None:
        """ start listen """

        if not self.started:
            self.started = not self.started
            self._start_ad()
        else:
            self.get_logger().info("stt is already running")

    def _start_ad(self) -> None:
        """ start listen with a thread"""

        while (self.__listen_thread is not None and self.__listen_thread.is_alive()):
            time.sleep(0.01)

        self.__listen_thread = CustomThread(
            target=self.__listen_thread_cb)
        self.__listen_thread.start()

    # STOP
    def __stop_ad_srv(
        self,
        req: Empty.Request,
        res: Empty.Response
    ) -> Empty.Response:
        """ service to stop listen

        Args:
            req(Empty.Request): empty
            res(Empty.Response): empty

        Returns:
            Empty.Response: empty
        """

        self.stop_ad()
        return res

    def stop_ad(self) -> None:
        """ stop listen """

        if self.started:
            self.started = not self.started
            self._stop_ad()
        else:
            self.get_logger().info("ad is already stopped")

    def _stop_ad(self) -> None:
        """ stop listen with a thread """
        # self._stream.stop_stream()
        # self._stream.close()
        # self._p.terminate()
        if (self.__listen_thread is not None and self.__listen_thread.is_alive()):
            self.__listen_thread.terminate()

        self.get_logger().info("stop listening")


def main():
    rclpy.init()
    sound_recognition_node = SoundRecongnitionNode()
    rclpy.spin(sound_recognition_node)
    sound_recognition_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
