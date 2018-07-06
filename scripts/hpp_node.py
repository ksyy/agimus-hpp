#!/usr/bin/env python
import sys, rospy, agimus_hpp.planning_request_adapter as pra, agimus_hpp.trajectory_publisher as tp
if "hpp-manipulation-server" in sys.argv:
    import agimus_hpp.manipulation.hpp_server_initializer as hsi
    import agimus_hpp.manipulation.planning_request_adapter as pra
    print "Launching manipulation client"
else:
    import agimus_hpp.hpp_server_initializer as hsi
    import agimus_hpp.planning_request_adapter as pra
    print "Launching default client"

rospy.init_node('hpp_server_connection')

_pra = pra.PlanningRequestAdapter("/joint_states")
_hsi = hsi.HppServerInitializer()
_tp = tp.HppOutputQueue ()

rospy.spin()
