import re
import sys
from os.path import isfile, join
from enum import Enum
import typing
from platformio.builder.tools.pioupload import AutodetectUploadPort
import os
import subprocess
import shutil
from subprocess import call, check_output

Import("env")
platform = env.PioPlatform()
board = env.BoardConfig()
mcu = board.get("build.mcu", "esp32")
# needed for later
AutodetectUploadPort(env)

listName = ["device", "device2"]


def compileAll(*args, **kwargs):
    print("Entrypoint")
    for i in range(len(listName)):
        env.Replace(PROGNAME="firmware_%s" % listName[i])


def buildCustom(source, target, env):
    pio_call = 'platformio run -e tasmota-{flash_mode} -t upload'.format(
        flash_mode=flash_mode)
    print("buildCustom")
