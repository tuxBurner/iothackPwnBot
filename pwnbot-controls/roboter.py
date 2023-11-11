from talker import Talker
import os
import time

os.system("mpremote run ./motor_code.py")
t = Talker()
t.call('motor_initialize()')
t.call('go_forward()')
time.sleep(5)
t.call('go_backward()')
time.sleep(5)
t.call('stop()')
t.call('motor_deinitialize()')