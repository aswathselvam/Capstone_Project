
import nxt
import time

import rospy
from std_msgs.msg import Int16

#brick = nxt.locator.find_one_brick(host='00:16:53:0F:9C:D9',name='nxtash', debug=True)
brick = nxt.locator.find_one_brick(debug=True)

motorSteer = nxt.Motor(brick, nxt.PORT_C)
motorDrive = nxt.Motor(brick, nxt.PORT_B)

def talker():
    pubSteer = rospy.Publisher('odo_steer', Int16, queue_size=3)
    pubDrive = rospy.Publisher('odo_drive', Int16, queue_size=3)
    rospy.init_node('node_pub_odo', anonymous=True)
    rate = rospy.Rate(80) # 80hz
    while not rospy.is_shutdown():
        pubSteer.publish(motorSteer.get_tacho().block_tacho_count)
        pubDrive.publish(motorDrive.get_tacho().block_tacho_count)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


