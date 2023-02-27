# roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

import rospy
import cv2
from cv2 import aruco
from std_msgs.msg import Int8

def talker():
    # Inisialisasi node
    rospy.init_node("aruco", anonymous=True)
    # Publisher mengirimkan data bertipe Int8
    pub = rospy.Publisher("detect_aruco", Int8, queue_size=10)
    rate = rospy.Rate(10) #10 Hz

    # Membaca webcam
    vid = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
    while(True):
        # Membaca tiap frame
        ret, frame = vid.read()
        # Convert ke gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Mendefinisikan ArUco dictionary yang digunakan
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        # Membuat objek parameter detektor
        parameters =  aruco.DetectorParameters_create()
        # Mendeteksi ArUco 
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        # Menggambar marker
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        # Menampilkan frame
        cv2.imshow('frame_marker', frame_markers)

        # Mengecek id ArUco yang terdeteksi dan mempublishnya
        if (ids != None):
            Ids = int(ids)
            print("ID ArUco :", Ids)
            pub.publish(Ids)

        # Menutup program jika "q" ditekan
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        rate.sleep()
        
    # Menutup vid
    vid.release()
    # Menutup semua window
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass