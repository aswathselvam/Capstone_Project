
import nxt
import time

import rospy
import roslib
from std_msgs.msg import Int16
import tf.transformations
from geometry_msgs.msg import Twist

#brick = nxt.locator.find_one_brick(host='00:16:53:0F:9C:D9',name='nxtash', debug=True)
brick = nxt.locator.find_one_brick(debug=True)

motorSteer = nxt.Motor(brick, nxt.PORT_C)
motorDrive = nxt.Motor(brick, nxt.PORT_B)

#motorDrive.run()
motorDrive.weak_turn(power=50,tacho_units=20)

def moveRobot(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    print(msg.linear.x, msg.linear.y, msg.linear.z)
    '''
    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands

    v_l = ...
    v_r = ...

    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    wheel_left.set_speed(v_l)
    wheel_right.set_speed(v_r)
    '''
    motorDrive.weak_turn(power=int(msg.linear.x),tacho_units=25)
    motorSteer.weak_turn(power=int(msg.angular.z),tacho_units=20)
    
        


def talker():
    pubSteer = rospy.Publisher('odo_steer', Int16, queue_size=3)
    pubDrive = rospy.Publisher('odo_drive', Int16, queue_size=3)
    
    rospy.Subscriber("cmd_vel", Twist, moveRobot,queue_size=1)
    
    rospy.init_node('nxt_controller', anonymous=True)
    rate = rospy.Rate(20) # 80hz
    while not rospy.is_shutdown():
        pubSteer.publish(motorSteer.get_tacho().block_tacho_count)
        pubDrive.publish(motorDrive.get_tacho().block_tacho_count)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


