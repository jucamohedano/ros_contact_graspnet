#!/usr/bin/env python3
"""
Custom Mask R-CNN model based on https://pytorch.org/tutorials/intermediate/torchvision_tutorial.html
"""
import rospy
import rospkg
import numpy as np
import os

from sensor_msgs.msg import PointCloud2
from ycb_mask_rcnn import YcbMaskRCNN
from std_msgs.msg import String
from contact_graspnet.srv import DetectObjects, DetectObjectsResponse

YCB_LABELS_FULL = [
            'ycb_063-a_marbles', 'ycb_052_extra_large_clamp', 'ycb_014_lemon',
            'ycb_073-c_lego_duplo', 'ycb_065-f_cups', 'ycb_029_plate',
            'ycb_007_tuna_fish_can', 'ycb_062_dice', 'ycb_061_foam_brick',
            'ycb_015_peach', 'ycb_010_potted_meat_can', 'ycb_022_windex_bottle',
            'ycb_016_pear', 'ycb_057_racquetball', 'ycb_063-b_marbles',
            'ycb_055_baseball', 'ycb_026_sponge', 'ycb_050_medium_clamp',
            'ycb_065-c_cups', 'ycb_032_knife', 'ycb_017_orange',
            'ycb_018_plum', 'ycb_065-d_cups', 'ycb_019_pitcher_base',
            'ycb_021_bleach_cleanser', 'ycb_056_tennis_ball', 'ycb_053_mini_soccer_ball',
            'ycb_042_adjustable_wrench', 'ycb_065-h_cups', 'ycb_072-b_toy_airplane',
            'ycb_072-a_toy_airplane', 'ycb_065-b_cups', 'ycb_040_large_marker',
            'ycb_025_mug', 'ycb_048_hammer', 'ycb_035_power_drill',
            'ycb_073-e_lego_duplo', 'ycb_011_banana', 'ycb_065-a_cups',
            'ycb_073-d_lego_duplo', 'ycb_008_pudding_box', 'ycb_037_scissors',
            'ycb_036_wood_block', 'ycb_004_sugar_box', 'ycb_072-e_toy_airplane',
            'ycb_073-f_lego_duplo', 'ycb_002_master_chef_can', 'ycb_058_golf_ball',
            'ycb_059_chain', 'ycb_024_bowl', 'ycb_006_mustard_bottle',
            'ycb_012_strawberry', 'ycb_031_spoon', 'ycb_005_tomato_soup_can',
            'ycb_009_gelatin_box', 'ycb_073-b_lego_duplo', 'ycb_073-a_lego_duplo',
            'ycb_070-a_colored_wood_blocks', 'ycb_003_cracker_box', 'ycb_054_softball',
            'ycb_038_padlock', 'ycb_072-c_toy_airplane', 'ycb_070-b_colored_wood_blocks',
            'ycb_073-g_lego_duplo', 'ycb_071_nine_hole_peg_test', 'ycb_033_spatula',
            'ycb_065-j_cups', 'ycb_028_skillet_lid', 'ycb_051_large_clamp',
            'ycb_065-e_cups', 'ycb_030_fork', 'ycb_072-d_toy_airplane',
            'ycb_077_rubiks_cube', 'ycb_043_phillips_screwdriver', 'ycb_065-i_cups',
            'ycb_044_flat_screwdriver', 'ycb_013_apple', 'ycb_027_skillet',
            'ycb_065-g_cups'
            ]


def object_is_attached(scene):
    attached_objects = scene.get_attached_objects(['object'])
    return len(attached_objects.keys()) > 0

def remove_object(scene):
    scene.remove_world_object('object')

def detach_object(move_group, scene):
    eef_link = move_group.get_end_effector_link()
    scene.remove_attached_object(eef_link, name='object')

MODEL_PATH = os.path.join(rospkg.RosPack().get_path('contact_graspnet'), 'src/contact_graspnet/utils/robocup.weights')
print(MODEL_PATH, rospkg.RosPack().get_path('contact_graspnet'))
mask_rcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)

def detect(msg):
    """
    Arguments:
        msg {DetectObjectsRequest} -- msg is empty
        
    Returns:
        detect_objects {DetectObjectsRespose} -- full_pcl, object_clouds, scores, labels_text
    """


    pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
    frame, pcl, boxes, clouds, scores, labels, labels_text, masks = mask_rcnn.detect(pclmsg, confidence=0.7, mask_confidence=0.5)
    labels_text = [String(label) for label in labels_text]

    print('labels: {}, scores: {}'.format(labels_text, scores))
    
    detect_objects = DetectObjectsResponse(pclmsg, clouds, scores, labels_text)
    
    
    return detect_objects


if __name__ == '__main__':
    rospy.init_node('maskRCNN_detection')
    s = rospy.Service('detect_objects', DetectObjects, detect)

    print('Ready to detect objects')
    rospy.spin()
