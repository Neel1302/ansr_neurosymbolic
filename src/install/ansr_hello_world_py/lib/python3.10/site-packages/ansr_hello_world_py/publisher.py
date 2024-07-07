from time import time

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import PIL
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch
from transformers import DetrImageProcessor, DetrForObjectDetection
import transformers

from adk_node.msg import TargetPerception

import scipy, gym, gurobipy, stable_baselines3, gym_minigrid, cvxpy, mlagents_envs, tqdm, matplotlib, shimmy, tikzplotlib


class Publisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.processor = DetrImageProcessor.from_pretrained("facebook/detr-resnet-50",
                                                            revision="no_timm")
        self.model = DetrForObjectDetection.from_pretrained("facebook/detr-resnet-50",
                                                            revision="no_timm")
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.model.to(self.device)
        self.model.eval()
        self.scene_subscription = self.create_subscription(
            Image,
            'adk_node/SimpleFlight/front_center_custom/Scene',
            self.detection_callback,
            1
        )
        self.timer = self.create_timer(10, self.timer_callback)

        self.dependency_publisher = self.create_publisher(String,
                                                          'adk_node/dependency', 1)
        self.perception_publisher = self.create_publisher(TargetPerception,
                                                          'adk_node/input/perception', 1)
        self.bounding_box_publisher = self.create_publisher(Image, 'adk_node/input/perception_bounding_box', 1)

    def timer_callback(self):
        msg = String()
        string = f'scipy version: {scipy.__version__}\n'
        string += f'gym version: {gym.__version__}\n'
        string += 'gurobipy version: '+str(gurobipy.gurobi.version()[0])+"."+str(gurobipy.gurobi.version()[1])+"."+str(gurobipy.gurobi.version()[2])+"\n" 
        string += f'stable_baselines3 version: {stable_baselines3.__version__}\n'
        string += f'cvxpy version: {cvxpy.__version__}\n'
        msg.data = string
        self.dependency_publisher.publish(msg)
        string = f'mlagents_envs version: {mlagents_envs.__version__}\n'
        string += f'tqdm version: {tqdm.__version__}\n'
        string += f'matplotlib version: {matplotlib.__version__}\n'
        string += f'shimmy version: {shimmy.__version__}\n'
        string += f'tikzplotlib version: {tikzplotlib.__version__}\n'
        msg.data = string
        self.dependency_publisher.publish(msg)

    def detection_callback(self, msg):
        cv_bridge = CvBridge()
        cv_image: np.ndarray = cv_bridge.imgmsg_to_cv2(msg)
        np_image = cv_image.astype(np.uint8)
        image = PIL.Image.fromarray(np_image, 'RGB')

        inputs = self.processor(images=image, return_tensors="pt")
        inputs = inputs.to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        target_sizes = torch.tensor([image.size[::-1]])
        results = self.processor.post_process_object_detection(outputs, target_sizes=target_sizes,
                                                               threshold=0.7)[0]


        for score, label_id, box in zip(results["scores"], results["labels"],
                                     results["boxes"]):
            left, top, right, bottom = box.tolist()
            left = int(left)
            top = int(top)
            right = int(right)
            bottom = int(bottom)
            cv.rectangle(cv_image, (left,top), (right, bottom), (255,0,0), 2)
            label = self.model.config.id2label[label_id.item()]
            cv.putText(cv_image, label, (left, bottom), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 1, cv.LINE_AA)
            perception_msg = TargetPerception()
            perception_msg.enter_or_leave = 1
            perception_msg.entity_id = label
            perception_msg.camera = 'front_center_custom'

            perception_msg.detection_time.sec = int(time())
            perception_msg.detection_time.nanosec = 0

            perception_msg.position.position.x = 0.0
            perception_msg.position.position.y = 0.0
            perception_msg.position.position.z = 0.0
            perception_msg.position.orientation.x = 0.0
            perception_msg.position.orientation.y = 0.0
            perception_msg.position.orientation.z = 0.0
            perception_msg.position.orientation.w = 0.0

            perception_msg.probability = round(score.item(), 2)
            self.perception_publisher.publish(perception_msg)

        bbox_img_msg = cv_bridge.cv2_to_imgmsg(cv_image)
        self.bounding_box_publisher.publish(bbox_img_msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

