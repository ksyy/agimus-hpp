#!/usr/bin/env python
import sys, rospy, agimus_hpp.planning_request_adapter as pra, agimus_hpp.trajectory_publisher as tp
import agimus_hpp.estimation as est

if "hpp-manipulation-server" in sys.argv:
    import agimus_hpp.manipulation.hpp_server_initializer as hsi
    print "Launching manipulation client"
else:
    import agimus_hpp.hpp_server_initializer as hsi
    print "Launching default client"

rospy.init_node('hpp_server_connection')

_est = est.Estimation ()
_hsi = hsi.HppServerInitializer()
_tp = tp.HppOutputQueue ()

rospy.spin()
