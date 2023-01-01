import rospy
from geometry_msgs.msg import Twist, Pose2D, Vector3
from std_msgs.msg import String
from std_msgs.msg import Int16

def joint_callback(servo):
    rospy.loginfo(f'i heard joint_1 {servo.x} joint_2 {servo.y} joint3 {servo.z}')
    values=Vector3()
    values.x=servo.x
    values.y=servo.y
    values.z=servo.z

    servo_pub.publish(values)



def node_initialization():
    rospy.init_node('arm', anonymous=True)
    rospy.Subscriber('/matlab_joints', Vector3, joint_callback)
    global servo_pub
    servo_pub=rospy.Publisher("servo_direction",Vector3, queue_size=10)

    # servo_pub=rospy.Publisher("servo_direction",String, queue_size=10)
    rospy.spin()

if _name=='main_':
    node_initialization()