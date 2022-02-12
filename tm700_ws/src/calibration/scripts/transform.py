import rospy
from sensor_msgs.msg import Image
def callback(img):
    print(img.header)
rospy.init_node("test_image")
rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback)

while True:
    pass
