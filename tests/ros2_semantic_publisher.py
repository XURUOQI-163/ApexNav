"""Opt-in ROS2 regression: inference latency must preserve acquisition header."""
import time
from unittest.mock import patch
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from omegaconf import OmegaConf
from plan_env.msg import SemanticScore
from real_world_test_example.real_world_test_habitat import RealWorldNode


def main():
    rclpy.init(domain_id=94)
    receiver=rclpy.create_node('semantic_publisher_probe')
    received=[]
    receiver.create_subscription(SemanticScore,'/semantic/score',received.append,10)
    bridge=CvBridge()
    try:
        for backend in ['clip','blip2']:
            cfg=OmegaConf.create({'semantic_model':backend,'llm':{
                'llm_answer_path':'unused','llm_response_path':'unused','llm_client':{'llm_client':'none'}}})
            node=RealWorldNode(cfg)
            try:
                node.label='chair';node.room='everywhere'
                deadline=time.monotonic()+3
                while not node.semantic_score_pub_.get_subscription_count() and time.monotonic()<deadline:
                    rclpy.spin_once(receiver,timeout_sec=.05)
                rgb=bridge.cv2_to_imgmsg(np.zeros((4,4,3),np.uint8),'rgb8')
                rgb.header.stamp=node.get_clock().now().to_msg();rgb.header.frame_id='world'
                depth=bridge.cv2_to_imgmsg(np.ones((4,4),np.float32),'32FC1');depth.header=rgb.header
                pose=Odometry();pose.header=rgb.header
                def delayed(image,label,room,selected):
                    assert selected==backend
                    time.sleep(.15)
                    return .234
                with patch('real_world_test_example.real_world_test_habitat.get_semantic_similarity',side_effect=delayed):
                    node.sync_value_callback(rgb,depth,pose)
                deadline=time.monotonic()+2
                while (not received or received[-1].backend!=backend) and time.monotonic()<deadline:
                    rclpy.spin_once(receiver,timeout_sec=.05)
                assert received[-1].backend==backend
                assert received[-1].header==rgb.header and received[-1].score==.234
                print(f'PASS: {backend} inference kept original image timestamp across 150 ms delay')
            finally:node.destroy_node()
    finally:receiver.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
