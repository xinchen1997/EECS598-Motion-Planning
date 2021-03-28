#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/rrtplugin')
try:
    env=Environment()
    env.Load('scenes/hw3.env.xml')
    rrtModule = RaveCreateModule(env,'rrtModule')
    print rrtModule.SendCommand('runRRT')
finally:
    RaveDestroy()
