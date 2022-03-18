#!/usr/bin/env python3
"""
Custom Mask R-CNN model based on https://pytorch.org/tutorials/intermediate/torchvision_tutorial.html
"""
import os
import numpy as np
import rospy
import cv2
import rospkg

from mask_rcnn import MaskRCNN # use this NN with YCB_DATASET
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor

import torch
torch.cuda.empty_cache()

import torchvision

# if torch.cuda.is_available():
#     DEVICE = 'cuda'
# else:
DEVICE = 'cpu' # FORCE CPU USAGE

from PIL import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge

def get_model_instance_segmentation(num_classes, pretrained=False):
    # load an instance segmentation model pre-trained pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=pretrained)

    # get number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    # replace the pre-trained head with a new one
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask,
                                                       hidden_layer,
                                                       num_classes)

    return model

class YcbMaskRCNN(MaskRCNN):
    def __init__(self, model_path, labels):
        self.model_path = None
        self.transform = torchvision.transforms.ToTensor()
        self.labels = labels
        self.load_model(model_path)

    def load_model(self, model_path):
        """ Load model
        loads Mask R-CNN model weights and labels.
        """
        # raise exception if files do not exist
        if not os.path.isfile(model_path):
            raise IOError('Model file not found: {}'.format(model_path))

        # load Mask R-CNN weights
        weights = torch.load(model_path)
        self.net = get_model_instance_segmentation(len(self.labels))
        self.net.load_state_dict(weights)

        self.model_path = model_path
        self.net.to(DEVICE)
        self.net.eval()

    def detect(self, pclmsg, confidence=0.7, mask_confidence=0.8, downsample=2):
        return super(YcbMaskRCNN, self).detect(pclmsg, self.model_path, confidence=0.7, mask_confidence=0.8, downsample=2)


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

if __name__ == '__main__':

    MODEL_PATH = os.path.join(rospkg.RosPack().get_path('contact_graspnet'), 'src/contact_graspnet/utils/robocup.weights')
    print (MODEL_PATH, rospkg.RosPack().get_path('contact_graspnet'))

    # colour stuff
    np.random.seed(69)
    COLOURS = np.random.randint(0, 256, (128,3))
    alpha = 0.5

    rospy.init_node('test')
    mask_rcnn = YcbMaskRCNN(MODEL_PATH, YCB_LABELS_FULL)

    while not rospy.is_shutdown():
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks = mask_rcnn.detect(pclmsg, confidence=0.8)
        # output point clouds
        for i, cloud in enumerate(clouds):
            pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
            pub.publish(cloud)

        for i, mask in enumerate(masks):
            if scores[i] > 0.6:
                label = labels[i]
                colour = COLOURS[label]

                # segmentation masks
                binary_mask = mask > 0.5
                frame_coloured = np.array((frame * (1-alpha) + colour * alpha), dtype=np.uint8)
                frame = np.where(binary_mask, frame_coloured, frame)

                # bboxes + info
                x1, y1, x2, y2 = [int(v) for v in boxes[i]]
                cv2.putText(frame, 'confidence: {:.2f}'.format(scores[i]), (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
                cv2.putText(frame, 'class: {}'.format(labels_text[i]), (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

        cv2.imshow('test', frame)
        cv2.waitKey(1)
