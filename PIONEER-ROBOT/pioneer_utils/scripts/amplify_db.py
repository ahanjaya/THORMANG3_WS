#!/usr/bin/env python3

import os
files = os.listdir('.')
db_gain = 3

for i in files:
    if i.endswith(".mp3"):
        command = 'lame --scale {} {} {}'.format(db_gain, i, 'amplified/'+i)
        os.system(command)
        print(i)

# print(files)