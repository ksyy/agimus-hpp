#!/usr/bin/env python
import rospy, hpp.corbaserver
from .client import HppClient
from dynamic_graph_bridge_msgs.msg import Vector
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf import TransformBroadcaster
from std_msgs.msg import Empty, UInt32
from std_srvs.srv import SetBool, SetBoolRequest
from math import cos, sin
from threading import Lock
import traceback
import ros_tools

### \brief Estimation based on HPP constraint solver.
##
## This class solves the following problem.
##
## Given:
## \li the current encoder values,
## \li tag poses (attached to objects) in the robot camera frame,
## \li a set of contraints encoding the semantic,
## estimate the complete robot configuration, including the robot base and, optionally, the object poses.
##
## The semantic of the problem (foot on the ground, object on the table...)
## can be expressed with HPP constraints. The cost is a mixture of:
## \li visual tag constraints: it penalizes the error between the tag position from CV and from estimation.
## \li current robot pose: it penalizes the error between the current robot pose from encoders and from estimation.
##
## There are two ways of specifying the semantic constraints:
## \li Core framework: set the ROS parameter "default_constraints" to a list of constraint names.
## \li Manipulation framework:
##     - the current state of the manipulation graph is estimated using the last configuration in
##       HPP server. It is a mixture of the result of the previous estimation and of the encoder values.
##     - if the current state cannot be estimated, it is assumed it has not changed since last iteration.
##     - the constraint of this state are used for estimation.
class Estimation(HppClient):
    ## Subscribed topics (prefixed by "/agimus")
    subscribersDict = {
            "estimation": {
                "request" : [Empty, "estimation" ],
                },
            "vision": {
                "tags": [TransformStamped, "get_visual_tag"],
                },
            }
    ## Provided services (prefixed by "/agimus")
    servicesDict = {
            "estimation": {
                "continuous_estimation" : [SetBool, "continuous_estimation" ],
                },
            }
    ## Published topics (prefixed by "/agimus")
    publishersDict = {
            "estimation": {
                # "estimation"          : [ Vector, 1],
                "semantic" : [ Vector, 1],
                "state_id" : [ UInt32, 1],
                },
            }

    def __init__ (self, continuous_estimation = False,
             joint_states_topic="/joint_states"):
        super(Estimation, self).__init__ (postContextId = "_estimation")

        self.subscribers = ros_tools.createSubscribers (self, "/agimus", self.subscribersDict)
        self.publishers  = ros_tools.createPublishers ("/agimus", self.publishersDict)
        self.services    = ros_tools.createServices (self, "/agimus", self.servicesDict)
        self.joint_state_subs = rospy.Subscriber (joint_states_topic, JointState, self.get_joint_state)
        self.locked_joints = []

        self.tf_pub = TransformBroadcaster()
        self.tf_root = "world"

        self.setHppUrl()

        self.mutex = Lock()

        self.robot_name = rospy.get_param("robot_name", "")

        self.last_stamp_is_ready = False
        self.last_stamp = rospy.Time.now()
        self.last_visual_tag_constraints = list()

        self.current_stamp = rospy.Time.now()
        self.current_visual_tag_constraints = list()

        self.continuous_estimation (SetBoolRequest(continuous_estimation))

    def continuous_estimation(self, msg):
        self.run_continuous_estimation = msg.data
	rospy.loginfo ("Run continuous estimation: {0}".format(self.run_continuous_estimation))
        return True, "ok"

    def spin (self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.run_continuous_estimation and self.last_stamp_is_ready:
                rospy.loginfo("Runnning estimation...")
                self.estimation()
            else:
                rospy.logdebug ("run continuous estimation."
                        +"run_continuous_estimation={0}, last_stamp_is_ready={1}"
                        .format(self.run_continuous_estimation, self.last_stamp_is_ready))
            rate.sleep()

    def estimation (self, msg=None):
        self.mutex.acquire()

        try:
            hpp = self._hpp()
            q_current = hpp.robot.getCurrentConfig()

            self._initialize_constraints (q_current)

            # The optimization expects a configuration which already satisfies the constraints
            projOk, q_projected, error = hpp.problem.applyConstraints (q_current)

            if projOk:
                optOk, q_estimated, error = hpp.problem.optimize (q_projected)
                if not optOk:
                    from numpy.linalg import norm
                    errNorm = norm(error)
                    if errNorm > 1e-2:
                      rospy.logwarn ("Optimisation failed ? error norm: {0}".format(errNorm))
                      rospy.logdebug ("estimated == projected: {0}".format(q_projected==q_estimated))
                    else:
                      rospy.loginfo ("Optimisation failed ? error norm: {0}".format(errNorm))
                    rospy.logdebug ("Error {0}".format(error))
                rospy.logdebug ("At {0}, estimated {1}".format(self.last_stamp, q_estimated))

                valid, msg = hpp.robot.isConfigValid (q_estimated)
                if not valid:
                    rospy.logwarn_throttle (1, "Estimation in collision: {0}".format(msg))

                self.publishers["estimation"]["semantic"].publish (q_estimated)

                self.publish_state (hpp)
            else:
                hpp.robot.setCurrentConfig (q_current)
                q_estimated = q_current
                rospy.logwarn ("Could not apply the constraints {0}".format(error))
        except Exception as e:
            rospy.logerr_throttle (1, str(e))
            rospy.logerr_throttle (1, traceback.format_exc())
        finally:
            self.last_stamp_is_ready = False
            self.mutex.release()

    ## Publish tranforms to tf
    # By default, only the child joints of universe are published.
    def publish_state (self, hpp):
        robot_name = hpp.robot.getRobotName()
        for jn in hpp.robot.getChildJointNames('universe'):
            links = hpp.robot.getLinkNames(jn)
            for l in links:
                T = hpp.robot.getLinkPosition (l)
                if l.startswith(robot_name):
                    name = l[len(robot_name)+1:]
                else:
                    name = l
                self.tf_pub.sendTransform (T[0:3], T[3:7], self.last_stamp, name, self.tf_root)
        # Publish the robot link as estimated.
        robot_joints = filter(lambda x: x.startswith(robot_name), hpp.robot.getAllJointNames())
        for jn in robot_joints:
            links = hpp.robot.getLinkNames(jn)
            for name in links:
                T = hpp.robot.getLinkPosition (name)
                self.tf_pub.sendTransform (T[0:3], T[3:7], self.last_stamp, name, self.tf_root)

    def _initialize_constraints (self, q_current):
        from CORBA import UserException
        hpp = self._hpp()

        hpp.problem.resetConstraints()

        if hasattr(self, "manip"): # hpp-manipulation:
            # Guess current state
            # TODO Add a topic that provides to this node the expected current state (from planning)
            manip = self._manip ()
            try:
                state_id = manip.graph.getNode (q_current)
                rospy.loginfo("At {0}, current state: {1}".format(self.last_stamp, state_id))
            except UserException:
                if hasattr(self, "last_state_id"): # hpp-manipulation:
                    state_id = self.last_state_id
                    rospy.logwarn("At {0}, assumed last state: {1}".format(self.last_stamp, state_id))
                else:
                    state_id = rospy.get_param ("default_state_id")
                    rospy.logwarn("At {0}, assumed default current state: {1}".format(self.last_stamp, state_id))
            self.last_state_id = state_id
            self.publishers["estimation"]["state_id"].publish (state_id)

            # copy constraint from state
            manip.problem.setConstraints (state_id, True)
            hpp.problem.addLockedJointConstraints("unused", self.locked_joints)
        else:
            # hpp-corbaserver: setNumericalConstraints
            default_constraints = rospy.get_param ("default_constraints")
            hpp.problem.addLockedJointConstraints("unused", self.locked_joints)
            hpp.problem.addNumericalConstraints ("constraints",
                    default_constraints,
                    [ 0 for _ in default_constraints ])

        # TODO we should solve the constraints, then add the cost and optimize.
        if len(self.last_visual_tag_constraints) > 0:
            rospy.loginfo("Adding {0}".format(self.last_visual_tag_constraints))
            hpp.problem.addNumericalConstraints ("unused", self.last_visual_tag_constraints,
                    [ 1 for _ in self.last_visual_tag_constraints ])
            hpp.problem.setNumericalConstraintsLastPriorityOptional (True)

    def get_joint_state (self, js_msg):
        from CORBA import UserException
        self.mutex.acquire()
        try:
            hpp = self._hpp()
            robot_name = hpp.robot.getRobotName()
            if len(robot_name) > 0: robot_name = robot_name + "/"
            for jn, q in zip(js_msg.name, js_msg.position):
                name = robot_name + jn
                jt = hpp.robot.getJointType(name)
                if jt.startswith("JointModelRUB"):
                    assert hpp.robot.getJointConfigSize(name) == 2, name + " is not of size 2"
                    qjoint = [cos(q), sin(q)]
                else:
                    assert hpp.robot.getJointConfigSize(name) == 1, name + " is not of size 1"
                    # Check joint bounds
                    bounds = hpp.robot.getJointBounds(name)
                    if q-bounds[0] < -1e-3 or q-bounds[1] > 1e-3:
                        rospy.logwarn_throttle(1, "Current state {1} of joint {0} out of bounds {2}"
                            .format(name, q, bounds))
                    qjoint = [min(bounds[1],max(bounds[0],q)),]
                hpp.problem.createLockedJoint ('lock_' + name, name, qjoint)
            if len(self.locked_joints) == 0:
                self.locked_joints = tuple(['lock_'+robot_name+n for n in js_msg.name])
	except UserException as e:
            rospy.logerr ("Cannot get joint state: {0}".format(e))
        finally:
            self.mutex.release()

    def get_visual_tag (self, ts_msg):
        stamp = ts_msg.header.stamp
        if stamp < self.current_stamp: return
        self.mutex.acquire()
        try:
            hpp = self._hpp()

            # Create a relative transformation constraint
            j1 = ts_msg.header.frame_id
            j2 = ts_msg.child_frame_id
            name = j1 + "_" + j2
            T = [ ts_msg.transform.translation.x,
                  ts_msg.transform.translation.y,
                  ts_msg.transform.translation.z,
                  ts_msg.transform.rotation.x,
                  ts_msg.transform.rotation.y,
                  ts_msg.transform.rotation.z,
                  ts_msg.transform.rotation.w,]
            # Compute scalar product between Z axis of camera and of tag.
            # TODO Add a weight between translation and orientation
            # It should depend on:
            # - the distance (the farthest, the hardest it is to get the orientation)
            distW = 1.
            # - the above scalar product (the closest to 0, the hardest it is to get the orientation)
            from hpp import Quaternion
            from numpy import array
            oriW = - Quaternion(T[3:]).transform(array([0,0,1]))[2]
            # - the tag size (for an orthogonal tag, an error theta in orientation should be considered
            #   equivalent to an position error of theta * tag_size)
            tagsize = 0.063 * 4 # tag size * 4
            s = tagsize * oriW * distW
            rospy.logdebug ("{} {}".format(oriW, s))
            names = ["P_"+name, "sO_"+name]
            hpp.problem.createPositionConstraint (names[0], j1, j2, T[:3], [0,0,0], [True,]*3)
            hpp.problem.createOrientationConstraint ("O_"+name, j1, j2, Quaternion(T[3:]).inv().toTuple(), [True,]*3)
            hpp.problem.scCreateScalarMultiply (names[1], s, "O_"+name)

            # If this tag is in the next image:
            if self.current_stamp < stamp:
                # Assume no more visual tag will be received from image at time current_stamp.
                self.last_stamp = self.current_stamp
                self.last_visual_tag_constraints = self.current_visual_tag_constraints
                # Reset for next image.
                self.current_stamp = stamp
                self.current_visual_tag_constraints = list()
                self.last_stamp_is_ready = True
            self.current_visual_tag_constraints.extend(names)
        finally:
            self.mutex.release()
