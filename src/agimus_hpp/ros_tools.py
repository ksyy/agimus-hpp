import rospy

## Wait indefinitely for a service.
## \param srv the service name
## \param time after which a warning is printed using rospy.logwarn
def wait_for_service (srv, time = 0.2):
    try:
        rospy.wait_for_service(srv, time)
    except rospy.ROSException:
        rospy.logwarn("Waiting for service: {0}".format(srv))
        rospy.wait_for_service(srv)
        rospy.logwarn("Service {0} found.".format(srv))

## Internal function. Use createSubscribers or createPublishers instead.
## \param subscribe boolean whether this node should subscribe to the topics.
##        If False, this node publishes to the topics.
def _createTopics (object, namespace, topics, subscribe):
    if isinstance(topics, dict):
        rets = dict ()
        for k, v in topics.items():
            rets[k] = _createTopics(object, namespace + "/" + k, v, subscribe)
        return rets
    else:
        if subscribe:
            try:
                callback = getattr(object, topics[1])
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(object.__class__.__name__, topics[1]))
            return rospy.Subscriber(namespace, topics[0], callback)
        else:
            return rospy.Publisher(namespace, topics[0], queue_size = topics[1])

## Create subscribers.
## \param object the object containing the callbacks.
## \param namespace prefix for the topic names
## \param topics a dictionary whose keys are topic names and values are a list [ Type, name_of_callback_in_object ].
##
## For instance:
## \code
## topics = { "foo" : { "topic1": [ std_msgs.msg.Empty, "function" ] }, }
## subscribers = createSubscribers (obj, "/bar", topics)
## # subscribers = { "foo" : {
## #                   "topic1": rospy.Subscriber ("/bar/foo/topic1", std_msgs.msg.Empty, obj.function)
## #                   }, }
## \endcode
def createSubscribers (object, namespace, topics):
    return _createTopics (object, namespace, topics, True)

## Create publishers.
## See \ref createSubscribers for a description of the parameters
##
## \param namespace prefix for the topic names
## \param topics a dictionary whose keys are topic names and values are a list [ Type, queue_size ].
def createPublishers (namespace, topics):
    return _createTopics (None, namespace, topics, False)

def _createServices (object, namespace, services, serve):
    """
    \param serve boolean whether this node should serve or use the topics.
    """
    if isinstance(services, dict):
        rets = dict ()
        for k, v in services.items():
            rets[k] = _createServices(object, namespace + "/" + k, v, serve)
        return rets
    else:
        if serve:
            try:
                callback = getattr(object, services[1])
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(object.__class__.__name__, services[1]))
            return rospy.Service(namespace, services[0], callback)
        else:
            wait_for_service (namespace)
            return rospy.ServiceProxy(namespace, services[0])

## Create rospy.Service.
## See \ref createSubscribers for a description of the parameters
## \param services a dictionary whose keys are topic names and values are a list [ Type, name_of_callback_in_object ].
def createServices (object, namespace, services):
    return _createServices (object, namespace, services, True)

## Create rospy.ServiceProxy.
## See \ref createSubscribers for a description of the parameters
## \param services a dictionary whose keys are topic names and values are a list [ Type, ].
def createServiceProxies (namespace, services):
    return _createServices (None, namespace, services, False)
