from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from deeplab.model import Model

class DeepLab_Node(Node):
    def __init__(self):
        super().__init__("DeepLab_Node")
        
        # Get model
        self.model = Model()

        # OpenCV bridge
        # Reference: https://wiki.ros.org/cv_bridge
        self.bridge = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(Image, "/input", self.callback, 20)
        self.subscription  # prevent unused variable warning

        # Publisher
        self.publisher = self.create_publisher(Image, "/output", 20)

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        result = self.model.inference(img)
        result_msg = self.bridge.cv2_to_imgmsg(result, encoding="rgb8")
        self.publisher.publish(result_msg)
