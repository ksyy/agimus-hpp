#!/usr/bin/env python
import rospy
import agimus_hpp.estimation as est

rospy.init_node('estimation')

_est = est.Estimation ()

rospy.spin()
