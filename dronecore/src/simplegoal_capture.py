# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
import tf2_geometry_msgs    
from tf.transformations import euler_from_quaternion
import json
import atexit

#rospy.init_node('flight')
data = {}
data['NavGoals'] = []
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), yaw_rate=0, speed=0.4, \
        frame_id='body', tolerance=0.2, auto_arm=False):

    res = navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, \
        frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)

def savedata():
    print("Saving Data")
    with open('navgoals.json', 'w') as outfile:
        json.dump(data, outfile)


atexit.register(savedata)

print ("Take off 1 meter")
navigate_wait(z=1, frame_id='body', auto_arm=True)
# Wait for 3 seconds
print("Done")
rospy.sleep(3)


# Fly forward 1 m
#navigate_wait(x=1, frame_id='body')

# Land
#land()


def goal_republish(posedata):

    #do the transformation
    euler = euler_from_quaternion([posedata.pose.orientation.x, posedata.pose.orientation.y, posedata.pose.orientation.z, posedata.pose.orientation.w])
    print("Received Simple Goal: x=",posedata.pose.position.x," y=",posedata.pose.position.y," z=",posedata.pose.position.z," yaw =",euler[2]," frameID=",posedata.header.frame_id)
    
    data['NavGoals'].append({
        'x': posedata.pose.position.x,
        'y': posedata.pose.position.y,
        'z': 1.0,
        'yaw': euler[2],
        'frameID': posedata.header.frame_id
    })

    navigate_wait( x=posedata.pose.position.x, y=posedata.pose.position.y, z=1, yaw=euler[2], frame_id=posedata.header.frame_id)
    print("Done")
    #rospy.sleep(3)




if __name__ == '__main__':

    rospy.init_node('republisher', anonymous=False)
    rospy.Subscriber("/move_base_simple/goal", tf2_geometry_msgs.PoseStamped, goal_republish)
    rospy.spin()