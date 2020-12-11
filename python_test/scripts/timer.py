#!/usr/bin/env python
# license removed for brevity
import rospy
import json
from std_msgs.msg import String

def timer():
    hzrate = 2
    
    pub = rospy.Publisher('sim_time', String, queue_size=100)
    rospy.init_node('timer', anonymous=True)
    rate = rospy.Rate(hzrate) # 10hz
    while not rospy.is_shutdown():
        time_string =  json.dumps({"time": rospy.get_time()})
        rospy.loginfo(time_string)
        pub.publish(time_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        timer()
    except rospy.ROSInterruptException:
        pass