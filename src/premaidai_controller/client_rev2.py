#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import datetime
import logging
import copy
import serial
import struct
import yaml
from collections import namedtuple

logger = logging.getLogger(__name__)


class Connector(serial.ReaderThread):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
