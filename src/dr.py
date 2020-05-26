#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ecn_mobile_control.cfg import GainsConfig

def callback(config, level):
    print(config)
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(GainsConfig, callback)
    rospy.spin()
