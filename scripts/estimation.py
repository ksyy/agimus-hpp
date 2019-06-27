#!/usr/bin/env python
import rospy, sys
import agimus_hpp.estimation as est

log_level = rospy.INFO
if "--debug" in sys.argv:
    log_level = rospy.DEBUG

rospy.init_node('estimation', log_level = log_level)

topic="/joint_states"
try:
  idx = sys.argv.index("--joint-state")
  try:
    topic = sys.argv[idx+1]
  except ValueError as e:
    rospy.logerr ("Could not set joint state topic: " + str(e))
except ValueError as e:
    pass

estimation_rate = 50
try:
  idx = sys.argv.index("--estimation-rate")
  try:
    estimation_rate = int (sys.argv[idx+1])
  except ValueError as e:
    rospy.logerr ("Could not set estimation rate: " + str(e))
except ValueError as e:
    pass
rospy.loginfo ("Estimation rate: " + str(estimation_rate))

visual_tags_enabled = True
if "--disable-visual-tag" in sys.argv:
    visual_tags_enabled = False

continuous_estimation = False
if "--continuous-estimation" in sys.argv:
    continuous_estimation = True

_est = est.Estimation (continuous_estimation=continuous_estimation,
        joint_states_topic=topic,
        visual_tags_enabled=visual_tags_enabled)
_est.estimation_rate = estimation_rate
_est.spin()
