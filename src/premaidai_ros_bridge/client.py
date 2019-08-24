#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import datetime
import logging
import copy
import serial
import struct
import threading
import yaml
from Queue import Queue
from collections import namedtuple

logger = logging.getLogger(__name__)

class CommandBase(object):
    u""" basical command class
    """
    CMD_GET_PARAM = 0x01            # プリメイドAIの情報を取得する（サーボ、バッテリー等）
    CMD_STOP = 0x05                 # 強制停止する
    CMD_SET_SERVO_POS= 0x18         # サーボの位置を設定する
    CMD_SET_SERVO_PARAM = 0x19      # サーボのパラメータを設定する
    CMD_SET_LED_CMD = 0x1E          # 胸のLEDを制御する
    CMD_PLAY_MOTION = 0x1F          # プリセットのMotionを再生する

    def __init__(self, command_type):
        self._command_type = command_type
        self._raw_response = None
        self._response = None

    @classmethod
    def _generate_check_byte(cls, data):
        ret = 0x00
        for idx, val in enumerate(data):
            if idx > 0:
                ret ^= val
            else:
                ret = val
        return ret

    @classmethod
    def _generate_binary_command(cls, send_command):
        # convert value to binary
        fmt = "{}B".format(len(send_command))
        binary_command = struct.pack(fmt, *send_command)
        return binary_command

    @classmethod
    def _generate_command(cls, command):
        ret = copy.copy(command)
        ret.append(cls._generate_check_byte(ret))
        ret.insert(0, len(ret) + 1)
        return ret

    def process_response(self, receive_data):
        self._raw_response = receive_data

    def get_raw_response(self):
        return self._raw_response

    def get_response(self):
        return self._response

    def get_command_type(self):
        return self._command_type


class MotionCommand(CommandBase):
    u""" Motionを実行するためのCommand
    """
    # プリメイドAIでサポートされているコマンド一覧
    DANCE = 1           # ダンス再生
    HOME = 61           # ホーム
    DOZO = 62           # どうぞ
    WAKUWAKU = 63       # ワクワク
    KOSHINITE = 64      # コシニテ
    YOUKOSO = 65        # ようこそ
    ONEGAI = 66         # おねがい
    BYEBYE = 67         # バイバイ
    RKISS = 68          # 右にキス
    LKISS = 69          # 左にキス
    GUN = 70            # ピストル
    KEIREI = 71         # 敬礼
    EN = 72             # え〜ん
    HART = 73           # ハート
    KIRA = 74           # キラッ
    ANATAHE = 75        # あなたへ
    MONKEY = 76         # もんきー
    GOGO = 77           # Gox2
    GUITER = 78         # エアギター
    RTURN = 79          # 右ターン
    LTURN = 80          # 左ターン

    # コマンドに対してのレスポンスエラー判定用
    _RES_EXECUTE_SUCCESS = 0x00
    _RES_EXECUTE_FAILURE = 0x80

    # レスポンスデータを保存するために使用
    Response = namedtuple("Response", ["command", "param", "checkbyte"])

    def __init__(self, motion_id):
        super(MotionCommand, self).__init__(self.CMD_PLAY_MOTION)
        self._motion_id = motion_id

    def generate_command(self):
        send_command = [self.CMD_PLAY_MOTION, 0x00, self._motion_id]
        send_command = self._generate_command(send_command)
        return self._generate_binary_command(send_command)

    def process_response(self, receive_data):
        # bnary_data をセット
        super(MotionCommand, self).process_response(receive_data)

        # process response
        self._response = MotionCommand.Response(*struct.unpack("3B", self._raw_response))

        # check command type
        if self._response.command != self.get_command_type():
            msg = "Invalid command id. excpected={} actual={}".format(self._response.command, self.get_command_type())
            raise ValueError(msg)

        # check error
        if self._response.param == self._RES_EXECUTE_SUCCESS:
            logger.debug("Success to play motion")

        elif self._response.param == self._RES_EXECUTE_FAILURE:
            msg = "Error to execute motion {}".format(self._motion_id)
            logger.warn(msg)
            raise RuntimeError(msg)

        else:
            msg = "Invalid response {}".format(self._response.param)
            logger.error(msg)
            raise RuntimeError(msg)


class ServoStatusCommand(CommandBase):
    u""" サーボの状態を取得するためのコマンド
    """
    _MAX_SERVO_NUM = 17
    _SERVO_DATA_LEN = 14
    _MIN_SERVO_ID = 1
    _MAX_SERVO_ID = 27

    Response = namedtuple(
        "ServoData",
        [
            "id",
            "enable",
            "actual_position",
            "offset",
            "command_position",
            "gyro_direction",
            "gyro_scale",
            "speed",
            "stretch"
        ])

    def __init__(self, start_id, servo_num):
        super(ServoStatusCommand, self).__init__(self.CMD_GET_PARAM)
        self._start_id = start_id
        self._servo_num = servo_num

        # check values
        if self._servo_num > self._MAX_SERVO_NUM:
            msg = "Servo num error. max servo num is {}".format(self._MAX_SERVO_NUM)
            raise ValueError(msg)

        # check id range
        if self._MIN_SERVO_ID > self._start_id or self._start_id > self._MAX_SERVO_ID:
            msg = "Servo id error. expect {} <= {} <= {}".format(self._MIN_SERVO_ID, self._start_id, self._MAX_SERVO_ID)
            raise ValueError(msg)

    def generate_command(self):
        send_command = [self.CMD_GET_PARAM, 0x00, 0x05, self._start_id, self._servo_num * self._SERVO_DATA_LEN]
        send_command = self._generate_command(send_command)
        return self._generate_binary_command(send_command)

    def process_response(self, receive_data):
        super(ServoStatusCommand, self).process_response(receive_data)

        self._response = []

        # check header
        command = struct.unpack('B', self._raw_response[0])[0]
        if command != self.get_command_type():
            msg = "Invalid command id. excpected={} actual={}".format(self.get_command_type(), command)
            raise ValueError(msg)

        # check data size
        # servo_data_len * servo_num + (parity + header=2)
        expect_size = self._SERVO_DATA_LEN * self._servo_num + 3
        if len(self._raw_response) != expect_size:
            msg = "Invalid ServoStatusCommand response. expect = {}, actual = {}".format(expect_size, len(self._raw_response))
            logger.error(msg)
            raise RuntimeError(msg)

        # parse servo data
        servo_data = self._raw_response[2:-1]
        for idx in range(self._servo_num):
            start_idx = self._SERVO_DATA_LEN * idx
            servo_binary_data = servo_data[start_idx:start_idx+self._SERVO_DATA_LEN]
            self._response.append(self.Response(*struct.unpack("2B5H2B", servo_binary_data)))


class ServoOffCommand(CommandBase):
    u""" サーボを脱力モードにするコマンド
        現在、脱力モードからの復帰コマンドは実装していない
    """
    _MIN_SERVO_ID = 1
    _MAX_SERVO_ID = 30

    # コマンドに対してのレスポンスエラー判定用
    _RES_EXECUTE_SUCCESS = 0x00             # OK
    _RES_EXECUTE_LENGTH_FAILURE = 0x08      # LENより短いご送信
    _RES_EXECUTE_OPTION_FAILURE = 0x10      # Option値異常

    # レスポンスデータを保存するために使用
    Response = namedtuple("Response", ["command", "param", "checkbyte"])

    def __init__(self):
        super(ServoOffCommand, self).__init__(self.CMD_SET_SERVO_POS)

    def generate_command(self):
        send_command = [self.CMD_SET_SERVO_POS, 0x00, 0x80]

        # すべて脱力状態 target_pos = 0 のコマンドを作成する
        for idx in range(self._MIN_SERVO_ID, self._MAX_SERVO_ID):
            send_command.extend([idx, 0x00, 0x00])
        send_command = self._generate_command(send_command)
        return self._generate_binary_command(send_command)

    def process_response(self, receive_data):
        # bnary_data をセット
        super(ServoOffCommand, self).process_response(receive_data)

        # process response
        self._response = ServoOffCommand.Response(*struct.unpack("3B", self._raw_response))

        # check command type
        if self._response.command != self.get_command_type():
            msg = "Invalid command id. excpected={} actual={}".format(self.get_command_type(), self._response.command)
            raise ValueError(msg)

        # check error
        if self._response.param == self._RES_EXECUTE_SUCCESS:
            return

        elif self._response.param == self._RES_EXECUTE_LENGTH_FAILURE:
            msg = "Invalid ServoOffCommand length."
            logger.warn(msg)
            raise RuntimeError(msg)

        elif self._response.param == self._RES_EXECUTE_OPTION_FAILURE:
            msg = "Invalid ServoOffCommand option."
            logger.warn(msg)
            raise RuntimeError(msg)

        else:
            msg = "Invalid response {}".format(self._response.param)
            logger.error(msg)
            raise RuntimeError(msg)


class ServoControlCommand(CommandBase):
    u""" サーボを直接制御するためのコマンド
    """
    Request = namedtuple("Request", ["id", "position"])

    # レスポンスデータを保存するために使用
    Response = namedtuple("Response", ["command", "param", "checkbyte"])

    # コマンドに対してのレスポンスエラー判定用
    _RES_EXECUTE_SUCCESS = 0x00             # OK
    _RES_EXECUTE_LENGTH_FAILURE = 0x08      # LENより短いご送信
    _RES_EXECUTE_OPTION_FAILURE = 0x10      # Option値異常

    def __init__(self, speed, requests):
        super(ServoControlCommand, self).__init__(self.CMD_SET_SERVO_POS)
        self._speed = speed
        self._requests = requests

    def generate_command(self):
        send_command = [self.CMD_SET_SERVO_POS, 0x00, self._speed]

        # すべて脱力状態 target_pos = 0 のコマンドを作成する
        for request in self._requests:
            send_command.append(request.id)
            send_command.extend([request.position & 0xFF, (request.position >> 0x08) & 0xFF])
        send_command = self._generate_command(send_command)
        return self._generate_binary_command(send_command)

    def process_response(self, receive_data):
        # bnary_data をセット
        super(ServoControlCommand, self).process_response(receive_data)

        # process response
        self._response = ServoOffCommand.Response(*struct.unpack("3B", self._raw_response))

        # check command type
        if self._response.command != self.get_command_type():
            msg = "Invalid command id. excpected={} actual={}".format(self.get_command_type(), self._response.command)
            raise ValueError(msg)

        # check error
        if self._response.param == self._RES_EXECUTE_SUCCESS:
            return

        elif self._response.param == self._RES_EXECUTE_LENGTH_FAILURE:
            msg = "Invalid ServoOffCommand length."
            logger.warn(msg)
            raise RuntimeError(msg)

        elif self._response.param == self._RES_EXECUTE_OPTION_FAILURE:
            msg = "Invalid ServoOffCommand option."
            logger.warn(msg)
            raise RuntimeError(msg)

        else:
            msg = "Invalid response {}".format(self._response.param)
            logger.error(msg)
            raise RuntimeError(msg)


class Sender(threading.Thread):
    def __init__(self, serial, connection_queue):
        super(Sender, self).__init__()
        self._serial = serial
        self._is_shutdown = False
        self._send_queue = Queue()
        self._connection_queue = connection_queue

    def shutdown(self):
        self._is_shutdown = True

    def send_command(self, request):
        self._send_queue.put(request)

    def run(self):
        while not self._is_shutdown:
            try:
                self._run()
            except Exception as err:
                logger.warn(str(err))

    def _run(self):
        while not self._send_queue.empty():
            # send data
            request = self._send_queue.get()
            send_data = request.command.generate_command()
            self._serial.write(send_data)
            self._connection_queue.put(request)
            time.sleep(0.1)


class Receiver(threading.Thread):
    def __init__(self, serial, connection_queue):
        super(Receiver, self).__init__()
        self._serial = serial
        self._is_shutdown = False
        self._recv_queue = Queue()
        self._connection_queue = connection_queue

    def shutdown(self):
        self._is_shutdown = True

    def run(self):
        while not self._is_shutdown:
            try:
                self._run()
            except Exception as err:
                logger.warn(str(err))

    def _run(self):
        # 接続用Queueに残っているデータの受信
        while not self._connection_queue.empty():
            try:
                # send data
                request = self._connection_queue.get()

                # receive data
                recv_data_len = struct.unpack("B", self._serial.read(1))[0] - 1
                recv_binary_data = self._serial.read(recv_data_len)

                # put data to receive queue
                request.command.process_response(recv_binary_data)

            except Exception as err:
                msg = "exception occured. msg = {}".format(err)
                logger.warn(msg)

            # call callback
            if request.callback:
                request.callback(request.command)

            # call event
            request.event.set()


class Connector(object):
    u""" プリメイドAIとコマンドのやり取りをするクラス
    """
    Request = namedtuple("Request", ["request_id" ,"command", "event", "callback"])

    def __init__(self, com, baudrate, timeout=None):
        self._connection_queue = Queue()
        self._sequence_id = 0
        self._serial = serial.Serial(com, baudrate, timeout=timeout)
        self._sender = Sender(self._serial, self._connection_queue)
        self._receiver = Receiver(self._serial, self._connection_queue)
        self._send_queue_lock = threading.RLock()

    def start(self):
        self._sender.start()
        self._receiver.start()

    def shutdown(self):
        # shutdonw thread
        self._sender.shutdown()
        self._sender.join()

        self._receiver.shutdown()
        self._receiver.join()

    def send_command(self, command, callback=None):
        with self._send_queue_lock:
            self._sequence_id += 1
            event = threading.Event()

            # set sent data
            self._sender.send_command(self.Request(self._sequence_id, command, event, callback))
            return event


class Controller(threading.Thread):
    u""" プリメイドAIを制御するためのインタフェースを提供するクラス
    """
    _ENCODER_MAX = 11500
    _ENCODER_MIN = 3500
    _SERVO_MAX_ANGLE = 135
    _SERVO_MIN_ANGLE = -135

    JointConfig = namedtuple("JointConfig", ["id", "name", "direction", "offset"])

    def __init__(self, serial_port, param_get_rate, config_file, joint_state_callback, publish_joint_state):
        # create connector
        super(Controller, self).__init__()
        self._connector = Connector(serial_port, 115200)
        self._is_shutdown = False
        self._sleep_time_sec = 1.0 / param_get_rate
        self._servo_status_callback_queue = Queue()
        self._joint_state_callback = joint_state_callback
        self._publish_joint_state = publish_joint_state

        # load config data for joint
        self._joint_name_config_dict = {}
        self._joint_id_config_dict = {}
        self._joint_names = []
        with open(config_file) as f:
            joint_configs = yaml.load(f)
            for config in joint_configs['joint_configs']:
                joint_config = Controller.JointConfig(**config)
                self._joint_id_config_dict[joint_config.id] = joint_config
                self._joint_name_config_dict[joint_config.name] = joint_config
                self._joint_names.append(joint_config.name)

        # start connector
        self._connector.start()

    def _servo_status_callback(self, cmd):
        self._servo_status_callback_queue.put(cmd)

    def _encoder_to_joint_angle(self, config, encoder_pos):
        angle = float(encoder_pos - self._ENCODER_MIN) / (self._ENCODER_MAX - self._ENCODER_MIN) * 270.0 - 135.0
        return (angle + config.offset) / 180.0 * math.pi * config.direction

    def _angle_to_encoder(self, config, angle):
        rate = (angle * 180.0 / math.pi * config.direction + 135.0) / 270.0
        enc_position = int(math.floor(rate * (self._ENCODER_MAX - self._ENCODER_MIN) + self._ENCODER_MIN))
        return enc_position

    def _speed_to_servo_speed(self, config, speed):
        return 0x80

    def shutdown(self):
        self._is_shutdown = True
        self._connector.shutdown()

    def run(self):
        while not self._is_shutdown:
            start_time = datetime.datetime.now()

            try:
                self._run_joint_state_publisher_loop()
            except Exception as err:
                logger.error(err)

            # calc sleep time
            diff_time = datetime.datetime.now() - start_time
            sleep_time = self._sleep_time_sec - diff_time.total_seconds()

            if sleep_time > 0:
                time.sleep(sleep_time)

    def _run_joint_state_publisher_loop(self):
        # JointStatusを定期的に取得するための関数
        if self._publish_joint_state:
            cmds = [ServoStatusCommand(1, 16), ServoStatusCommand(17, 17)]

            # 送信した分のイベントを保存
            [self._connector.send_command(cmd, self._servo_status_callback) for cmd in cmds]

            # データを受信していた場合はまとめて送信
            while self._servo_status_callback_queue.qsize() > 1:
                cmds = [self._servo_status_callback_queue.get(), self._servo_status_callback_queue.get()]

                # gather response
                # TODO サーボIDをの判別が必要
                joint_status_list = []
                [joint_status_list.extend(cmd.get_response()) for cmd in cmds]

                # convert encoder value to joint angle
                ret = {}
                for joint_status in joint_status_list:
                    if joint_status.id in self._joint_id_config_dict:
                        config = self._joint_id_config_dict[joint_status.id]
                        name = config.name
                        ret[name] = self._encoder_to_joint_angle(config, joint_status.actual_position)

                # call joint status calback
                self._joint_state_callback(ret)

    def power_off(self):
        cmd = ServoOffCommand()
        event = self._connector.send_command(cmd)
        event.wait()

    def set_joint_positions(self, joint_commands):
        requests = []
        for joint_name, position in joint_commands.items():
            if joint_name in self._joint_name_config_dict:
                # get config data from dict
                config = self._joint_name_config_dict[joint_name]
                enc_pos = self._angle_to_encoder(config, position)
                enc_speed = self._speed_to_servo_speed(config, 0x00)
                request = ServoControlCommand.Request(config.id, enc_pos)
                requests.append(request)

        # send command and get response
        cmd = ServoControlCommand(0x80, requests)
        event = self._connector.send_command(cmd)
        # event.wait()

    def execute_motion(self, motion_id):
        cmd = MotionCommand(motion_id)
        event = self._connector.send_command(cmd)
        event.wait()
        return True

    def get_joint_names(self):
        return self._joint_names