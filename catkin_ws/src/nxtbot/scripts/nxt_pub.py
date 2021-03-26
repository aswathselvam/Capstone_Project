
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
stackSteer = []
stackDrive = 0
frontback=1

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
    #motorDrive.weak_turn(power=int(msg.linear.x),tacho_units=25)
    #motorSteer.weak_turn(power=int(msg.angular.z),tacho_units=20)

def steer(msg):
    stackSteer.append(msg.data)


def drive(msg):
    stackDrive=msg.data
    frontback=1 if stackDrive>0 else -1 : 

    print(msg.data)


def talker():
    pubSteer = rospy.Publisher('nxt/odo_steer', Int16, queue_size=3)
    pubDrive = rospy.Publisher('nxt/odo_drive', Int16, queue_size=3)
    
    rospy.Subscriber("cmd_vel", Twist, moveRobot,queue_size=1)
    rospy.Subscriber("nxt/steer_motor", Int16, steer,queue_size=1)
    rospy.Subscriber("nxt/drive_motor", Int16, drive,queue_size=1)

    rospy.init_node('nxt_pub', anonymous=True)
    rate = rospy.Rate(1) # 80hz

    prevstackSteer=0
    while not rospy.is_shutdown():
        pubSteer.publish(-1*motorSteer.get_tacho().block_tacho_count)
        pubDrive.publish(motorDrive.get_tacho().block_tacho_count)

        if (len(stackSteer)>0):
            motorSteer.weak_turn(power=50,tacho_units=stackSteer[0])
            stackSteer.pop()

        if abs(stackDrive)>10:
            motorDrive.weak_turn(power=50,tacho_units=stackDrive)
            stackDrive=stackDrive + frontback* 10

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


