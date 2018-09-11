#!/usr/bin/env python
import rospy, sys
import agimus_hpp.estimation as est

log_level = rospy.INFO
if "--debug" in sys.argv:
    log_level = rospy.DEBUG

rospy.init_node('estimation', log_level = log_level)

try: 
  idx = sys.argv.index("--joint-state")
  topic = sys.argv[idx+1]
except ValueError:
  topic="/joint_states"

if "--continuous-estimation" in sys.argv:
  _est = est.Estimation (True, joint_states_topic=topic)
else:
  _est = est.Estimation (False, joint_states_topic=topic)

_est.spin()
