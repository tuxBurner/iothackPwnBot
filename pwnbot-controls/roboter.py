from talker import Talker
import os

os.system("mpremote run ./motor_code.py")
t = Talker()
t.call('motor_initialize()')
t.call('motor_start()')
t.call('motor_stop()')