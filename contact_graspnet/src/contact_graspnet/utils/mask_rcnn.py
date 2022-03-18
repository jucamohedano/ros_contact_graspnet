#!/usr/bin/env python3
import numpy as np
import os.path

import torch
import torchvision
import torchvision.transforms as transforms

# if torch.cuda.is_available():
#     DEVICE = 'cuda'
# else:
DEVICE = 'cpu' # use only CPU

from PIL import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge

# COCO names from pytorch.org
COCO_INSTANCE_CATEGORY_NAMES = [
    '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
    'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
    'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
    'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]



def pclmsg_to_pcl_cv2_imgmsg(pclmsg):
    # extract the xyz values from 32FC1
    pcl = np.fromstring(pclmsg.data, dtype=np.uint8)
    pcl = pcl.reshape(pclmsg.height, pclmsg.width, -1)

    # extract the rgb values from 32FC1
    frame = np.fromstring(pclmsg.data, dtype=np.uint8)
    frame = frame.reshape(pclmsg.height, pclmsg.width, 32)
    frame = frame[:,:,16:19].copy()

    # imgmsg
    bridge = CvBridge()
    # imgmsg = bridge.cv2_to_imgmsg(frame, encoding='rgb8')

    return pcl, frame#, imgmsg



def mask_rcnn_transform():
    return transforms.Compose([
        transforms.ToTensor(),
    ])



class MaskRCNN(object):
    def __init__(self, model_path):
        self.model_path = None
        self.transform = mask_rcnn_transform()
        self.load_model(model_path)


    def load_model(self, model_path):
        """ Load model
        loads Mask R-CNN model weights and labels.
        """
        if model_path == '_default':
            self.net = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=True)
            self.labels = COCO_INSTANCE_CATEGORY_NAMES
            self.net.eval()
        else:
            # raise exception if files do not exist
            if not os.path.isdir(model_path):
                raise IOError('Model directory not found: {}'.format(model_path))
            if not os.path.isfile(os.path.join(model_path, 'model.pth')):
                raise IOError('model.pth not found in model directory: {}'.format(model_path))
            if not os.path.isfile(os.path.join(model_path, '_labels.txt')):
                raise IOError('_labels.txt not found in model directory: {}'.format(model_path))

            # load Mask R-CNN weights
            weights = torch.load(os.path.join(self.model_path, 'model.pth'))
            self.net = torchvision.models.detection.maskrcnn_resnet50_fpn()
            self.net.load_state_dict(weights)

            # parse text labels
            self.labels = open(os.path.join(self.model_path, '_labels.txt')).read().strip().split('\n')

        self.model_path = model_path
        self.net.to(DEVICE)
        self.net.eval()


    def forward(self, model_path, frame):
        """ Forward pass
        runs a forward pass of Mask R-CNN on an input frame.

        PARAMS: model_path  path to directory containing "model.pth"
                frame       bgr cv2 image

        RETURN: boxes       bounding boxes x1,y1,x2,y2
                labels      numbered labels
                labels_txt  text labels
                masks       segmentation masks for each detection
        """
        # load model if it is not already up
        if not self.model_path == model_path:
            self.load_model(model_path)

        assert self.net is not None
        
        # preprocess
        image = Image.fromarray(frame[:, :, ::-1]) # rgb -> bgr -> PIL
        image = torch.stack([self.transform(image)]).to(DEVICE)

        # net forward
        predictions = self.net(image)
        boxes = predictions[0]['boxes'].cpu().detach().numpy()
        labels = predictions[0]['labels'].cpu().detach().numpy()
        scores = predictions[0]['scores'].cpu().detach().numpy()
        masks = predictions[0]['masks'].permute(0,2,3,1).cpu().detach().numpy()

        # get text labels
        labels_txt = [self.labels[label] for label in labels]

        return boxes, labels, labels_txt, scores, masks


    def detect(self, pclmsg, model_path='_default', confidence=0.7, mask_confidence=0.8, downsample=2):
        pcl, frame = pclmsg_to_pcl_cv2_imgmsg(pclmsg)
        pred_boxes, pred_labels, pred_labels_text, pred_scores, pred_masks = self.forward(model_path, frame)

        # print('pred_boxes: {}, pred_labels: {}, pred_labels_text: {}, pred_scores: {}, pred_masks: {}'.format(len(pred_boxes), len(pred_labels), len(pred_labels_text), len(pred_scores), len(pred_masks)))

        # results
        labels = []
        labels_text = []
        scores = []
        masks = []
        boxes = []
        clouds = []

        for i, label in enumerate(pred_labels):
            if pred_scores[i] > confidence:
                # downsample everything
                mask = pred_masks[i]
                frame_ds = frame[::downsample, ::downsample, :]
                pcl_ds = pcl[::downsample, ::downsample, :]
                mask_ds = mask[::downsample, ::downsample]

                # get indices of mask
                binary_mask = mask_ds > mask_confidence
                binary_mask = binary_mask.flatten()
                indices = np.argwhere(binary_mask).flatten()

                # extract segmented detection
                frame_out = np.take(frame_ds.reshape(frame_ds.shape[0] * frame_ds.shape[1], -1), indices, axis=0)
                pcl_out = np.take(pcl_ds.reshape(pcl_ds.shape[0] * pcl_ds.shape[1], -1), indices, axis=0)

                # create pcl
                pclmsg_out = PointCloud2()
                pclmsg_out.header       = pclmsg.header
                pclmsg_out.height       = pcl_out.shape[0]
                pclmsg_out.width        = 1
                pclmsg_out.fields       = pclmsg.fields
                pclmsg_out.is_bigendian = pclmsg.is_bigendian
                pclmsg_out.point_step   = pclmsg.point_step
                pclmsg_out.row_step     = pcl_out.shape[1]
                pclmsg_out.is_dense     = pclmsg.is_dense
                pclmsg_out.data         = pcl_out.flatten().tostring()

                # append results
                masks.append(mask)
                boxes.append(pred_boxes[i])
                clouds.append(pclmsg_out)
                scores.append(pred_scores[i])
                labels.append(pred_labels[i])
                labels_text.append(pred_labels_text[i])
            # else:
            #     masks.append(None)
            #     boxes.append(None)
            #     clouds.append(PointCloud2())
        
        return frame, pcl, boxes, clouds, scores, labels, labels_text, masks



if __name__ == '__main__':
    import rospy
    import cv2

    # colour stuff
    np.random.seed(69)
    COLOURS = np.random.randint(0, 256, (128,3))
    alpha = 0.5

    rospy.init_node('test')
    mask_rcnn = MaskRCNN('_default')

    while not rospy.is_shutdown():
        pclmsg = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)
        frame, pcl, boxes, clouds, scores, labels, labels_text, masks = mask_rcnn.detect(pclmsg)

        # output point clouds
        for i, cloud in enumerate(clouds):
            pub = rospy.Publisher('segmentations/{}'.format(i), PointCloud2, queue_size=1)
            pub.publish(cloud)

        for i, mask in enumerate(masks):
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
