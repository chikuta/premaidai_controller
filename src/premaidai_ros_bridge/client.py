#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
import copy
import serial
import struct
import threading
import abc
from Queue import Queue
from collections import namedtuple

logger = logging.getLogger(__name__)

class CommandBase(object):
    u""" basical command class
    """
    __metaclass__ = abc.ABCMeta
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
    LTURN = 80          # 左ターン:w

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
            msg = "Invalid command id. excpected={} actual={}".format(self._generate_command(), self._response.command)
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

        # check data size
        # servo_data_len * servo_num + (parity + header=2)
        expect_size = self._SERVO_DATA_LEN * self._servo_num + 3
        if len(self._raw_response) != expect_size:
            msg = "Invalid SErvoStatusCommand response. expect = {}, actual = {}".format(expect_size, len(self._raw_response))
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


class Connector(threading.Thread):
    def __init__(self, com, baudrate, timeout=None):
        super(Connector, self).__init__()
        self._receive_queue = Queue()
        self._send_queue = Queue()
        self._is_shutdown = False
        self._sequence_id = 0
        self._serial = serial.Serial(com, baudrate, timeout=timeout)

    def shutdown(self):
        self._is_shutdown = True
        self._serial.close()

    def run(self):
        while not self._is_shutdown:
            try:
                self._run()
            except Exception as err:
                pass

    def _run(self):
        while not self._send_queue.empty():
            # send data
            command = self._send_queue.get()
            send_data = command.generate_command()
            self._serial.write(send_data)

            # receive data
            recv_data_len = struct.unpack("B", self._serial.read(1))[0] - 1
            recv_binary_data = self._serial.read(recv_data_len)

            # put data to receive queue
            command.process_response(recv_binary_data)
            self._receive_queue.put(command)

    def send_command(self, command):
        self._send_queue.put(command)

    def get_response(self):
        return self._receive_queue.get()
