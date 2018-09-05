#!/usr/bin/env python
import rospy, sys
import agimus_hpp.estimation as est

log_level = rospy.INFO
if "--debug" in sys.argv:
    log_level = rospy.DEBUG

rospy.init_node('estimation', log_level = log_level)

if "--continuous-estimation" in sys.argv:
  _est = est.Estimation (True)
else:
  _est = est.Estimation (False)

_est.spin()
