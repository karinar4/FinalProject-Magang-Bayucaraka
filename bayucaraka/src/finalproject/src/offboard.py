import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import cv2
import numpy as np
from cv2 import aruco
import math
from std_msgs.msg import Int8

current_state = State()

# Memperbarui nilai current_state
def state_cb(msg):
    global current_state
    current_state = msg

current_x = 0.0
current_y = 0.0
current_z = 0.0

# Memperbarui nilai koordinat posisi saat ini
def pose_cb(msg):
    global current_x, current_y, current_z
    current_x = msg.pose.position.x
    current_y = msg.pose.position.y
    current_z = msg.pose.position.z

# Memeriksa apakah sudah mencapai posisi yang diinginkan
def check(desired_x, desired_y, desired_z):
    if math.fabs(current_x - desired_x) <= 0.01 and math.fabs(current_y - desired_y) <= 0.01 and math.fabs(current_z - desired_z) <= 0.01:
        return True
    else:
        return False

# Menjalankan drone sesuai ArUco yang terdeteksi
def detect_aruco_cb(data):
    # Menyimpan perintah arming dan disarming
    arm_cmd = CommandBoolRequest()
    disarm_cmd = CommandBoolRequest()

    time = 0
    
    if (data.data == 0):
        rospy.loginfo("Now let's arm and take off")

        pose.pose.position.x = current_x
        pose.pose.position.y = current_y
        pose.pose.position.z = 2

        arm_cmd.value = True

        last_req = rospy.Time.now()

        # Arming
        while(not rospy.is_shutdown()):
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

            if check(current_x, current_y, 2.0):
                break

            local_pos_pub.publish(pose)
            rate.sleep()
        
        rospy.loginfo("Done...")
                
    elif (data.data == 1):
        rospy.loginfo("Moving toward the positive x-axis")

        vel.twist.linear.x = 1
        vel.twist.linear.y = 0
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel)

    elif (data.data == 2):
        rospy.loginfo("Moving toward the positive y-axis")

        vel.twist.linear.x = 0
        vel.twist.linear.y = 1
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel)

    elif (data.data == 3):
        rospy.loginfo("Moving toward the positive z-axis")

        vel.twist.linear.x = 0
        vel.twist.linear.y = 0
        vel.twist.linear.z = 1
        cmd_vel_pub.publish(vel)

    elif (data.data == 4):
        rospy.loginfo("Moving toward the negative x-axis")

        vel.twist.linear.x = -1
        vel.twist.linear.y = 0
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel)

    elif (data.data == 5):
        rospy.loginfo("Moving toward the negative y-axis")

        vel.twist.linear.x = 0
        vel.twist.linear.y = -1
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel)

    elif (data.data == 6):
        rospy.loginfo("Moving toward the negative z-axis")

        vel.twist.linear.x = 0
        vel.twist.linear.y = 0
        vel.twist.linear.z = -1
        cmd_vel_pub.publish(vel)

    elif (data.data == 7):
        rospy.loginfo("Moving in a zig-zag pattern toward the positive y-axis")

        time = rospy.Time.now().to_sec()

        vel.twist.linear.x = math.cos(2 * time)
        vel.twist.linear.y = 1
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel) 

    elif (data.data == 8):
        rospy.loginfo("Moving in a zig-zag pattern toward the negative y-axis")

        time = rospy.Time.now().to_sec()

        vel.twist.linear.x = math.cos(2 * time)
        vel.twist.linear.y = -1
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel)

    elif (data.data == 9):
        rospy.loginfo("Moving in a zig-zag pattern toward the positive x-axis")

        time = rospy.Time.now().to_sec()

        vel.twist.linear.x = 1
        vel.twist.linear.y = math.sin(2 * time)
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel)

    elif (data.data == 10):
        rospy.loginfo("Moving in a zig-zag pattern toward the negative x-axis")

        time = rospy.Time.now().to_sec()

        vel.twist.linear.x = -1
        vel.twist.linear.y = math.sin(2 * time)
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel)

    elif (data.data == 11):
        rospy.loginfo("Moving in a circular pattern")

        time = rospy.Time.now().to_sec()

        vel.twist.linear.x = math.cos(0.5 * time)
        vel.twist.linear.y = math.sin(0.5 * time)
        vel.twist.linear.z = 0
        cmd_vel_pub.publish(vel)

    elif (data.data == 12):
        rospy.loginfo("Now let's land and disarm")
                
        vel.twist.linear.x = 0
        vel.twist.linear.y = 0
        vel.twist.linear.z = -0.5

        while check(current_x, current_y, 0.0) == False:   
            cmd_vel_pub.publish(vel)
            rate.sleep()
        
        disarm_cmd.value = False

        # Disarming
        if(arming_client.call(disarm_cmd).success == True):
            rospy.loginfo("Vehicle disarmed")
        
        if not current_state.armed:
            rospy.loginfo("Done...")

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    cmd_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    pose_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_cb)
    detect_aruco_sub = rospy.Subscriber("detect_aruco", Int8, detect_aruco_cb)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20) # 20 Hz

    # Menunggu koneksi dengan Flight Controller
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Menyimpan perintah untuk mengaktifkan mode "OFFBOARD"
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Menyimpan posisi
    pose = PoseStamped()
    # Menyimpan kecepatan
    vel = TwistStamped()

    last_req = rospy.Time.now()

    # Mengaktifkan mode "OFFBOARD"
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
                break
            
            last_req = rospy.Time.now()

        rate.sleep()

    rospy.spin()