from geometry_msgs.msg import Vector3, Quaternion, Transform

def listToVector3(l):
    return Vector3 (l[0], l[1], l[2])
def listToQuaternion(l):
    return Quaternion (l[0], l[1], l[2], l[3])
def listToTransform(l):
    return Transform(listToVector3(l[0:3]), listToQuaternion(l[3:7]))

def hppPoseToSotTransRPY(pose):
    from hpp import Quaternion
    q = Quaternion(pose[3:7])
    return pose[0:3] + q.toRPY().tolist()
