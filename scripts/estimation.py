#!/usr/bin/env python
import sys
import argparse

import rospy

import agimus_hpp.estimation as est

parser = argparse.ArgumentParser()

parser.add_argument("-d", "--debug", action="store_true", help="Set logging level to Debug")
parser.add_argument("--disable_visual_tags", action="store_true", help="Disable the usage of visual tags")
parser.add_argument("--continuous_estimation", action="store_true", help="Enable continuous estimation")
parser.add_argument(
    "--joint_state", type=str, default="/joint_states", help="Name of the ROS Topic publishing the joint states"
)
parser.add_argument("--estimation_rate", type=int, default=50, help="Frequency of the estimation publishing")

args = parser.parse_args()

log_level = rospy.DEBUG if args.debug else rospy.INFO
rospy.init_node("estimation", log_level=log_level)
rospy.loginfo("Estimation rate: " + str(args.estimation_rate))

_est = est.Estimation(
    continuous_estimation=args.continuous_estimation,
    joint_states_topic=args.joint_state,
    visual_tags_enabled=not args.disable_visual_tags,
)
_est.estimation_rate = args.estimation_rate
_est.spin()
