import rclpy
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from .submodules.detect import detect_apriltag

class Camera(Node):
    def __init__(self):
        super().__init__('Image_Subscriber')
        self.image_subscriber = self.create_subscription(
                            Image, 
                            "/moss_bot0/rgb_cam/image_raw", 
                            self.image_callback, 
                            10)
        
        self.image_subscriber

    def image_callback(self, msg):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rclpy.logger.error(e)
        
        image = cv_image

        resized_image = cv2.resize(image, (640, 480)) 

        tags = detect_apriltag(resized_image)

        for tag in tags:
            start_point = (int(tag.corners[3][0]),int(tag.corners[3][1]))
            end_point = (int((tag.corners[1][0])),int(tag.corners[1][1]))
            print(start_point)
            resized_image = cv2.rectangle(resized_image, start_point, end_point, (0,255,0), 3)
                
        cv2.imshow('Robot Camera', resized_image)

        cv2.waitKey(3)

        #k = cv2.waitKey(1)
                
        # Esc
        #if k%256 == 27:
            #break
                
        # Space
        #elif k%256 == 32:
            #print('hit me PAUL')
            #cv2.imwrite('testimage2.jpg', image)

def main(args = None):
    rclpy.init(args = args)
    image_subscriber = Camera()
  
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        rclpy.logger.info("Shutting down")
  
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()