#!/usr/bin/env python3

import argparse
from functools import partial

import rospy

from tflite_detector.msg import Detect
from geometry_msgs.msg import Pose, Quaternion, Vector3

# from here: https://www.pyimagesearch.com/2016/11/07/intersection-over-union-iou-for-object-detection/
def bb_intersection_over_union(boxA, boxB):
	# determine the (x, y)-coordinates of the intersection rectangle
	xA = max(boxA[0], boxB[0])
	yA = max(boxA[1], boxB[1])
	xB = min(boxA[2], boxB[2])
	yB = min(boxA[3], boxB[3])
	# compute the area of intersection rectangle
	interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
	# compute the area of both the prediction and ground-truth
	# rectangles
	boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
	boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
	# compute the intersection over union by taking the intersection
	# area and dividing it by the sum of prediction + ground-truth
	# areas - the interesection area
	iou = interArea / float(boxAArea + boxBArea - interArea)
	# return the intersection over union value
	return iou

class Tracker(object):
    yaw = 0
    pitch = 0
    publisher = None
    fov = [48.8, 48.8]

    box = [0,0,1,1]
    score = 1
    initial = True

    def __init__(self, yaw, pitch, publisher):
        self.yaw = yaw
        self.pitch = pitch
        self.pub = publisher


    def process_detection(self, detect):
        if detect.num == 0:
            return

        max_iou = 0
        max_index = -1

        for i in range(detect.num):
            iou = bb_intersection_over_union(self.box, detect.boxes[i].coords)

            if iou > max_iou:
                max_iou = iou
                max_index = i
                #print('found: {}'.format(detect.boxes[i]))
                #print('cvrtd: {}'.format(boxes[i]))

        if max_index < 0 and detect.scores[0] > 0.5:
            self.box = detect.boxes[0].coords 
            self.score = detect.scores[0]

            self.adjust_position()
        elif detect.scores[max_index] > 0.5:
            self.box = detect.boxes[max_index].coords
            self.score = detect.scores[max_index]
                
            self.adjust_position()

    def process_position(self, position):
        self.yaw = position.z
        self.pitch = position.y
        self.initial = False

    def adjust_position(self):
        # find the center of the box
        #print('adjusting to: {}'.format(self.box))
        x = 0.5 * (self.box[3] + self.box[1]) - 0.5
        y = 0.5 * (self.box[2] + self.box[0]) - 0.5

        #print('box center: {} {}'.format(x, y))

        # multiply by the field of view
        dx = -x * self.fov[0]
        dy = y * self.fov[1]

        # soften the values
        dx /= 3 
        dy /= 3 

        # add that to the current yaw
       
        #TODO: these bounds checking should be replaced by feedback from the pantilt camera's actual position.  How do you query a node to get data from it?
        if self.pitch + dy >= 0 and self.pitch + dy < 180:
            self.pitch += dy
        if self.yaw + dx >= 0 and self.yaw + dx < 180:
            self.yaw += dx

        if not self.initial:
            #print('positioning: {} {}'.format(self.pitch, self.yaw))

            self.pub.publish(Vector3(0, self.pitch, self.yaw))

def track(args):
    rospy.init_node('object_tracker', anonymous=True)

    pub = rospy.Publisher('pantiltPose', Vector3, queue_size=10)

    tracker = Tracker(args.initial_yaw, args.initial_pitch, pub)

    rospy.Subscriber('detections', Detect, lambda d: tracker.process_detection(d)) 
    rospy.Subscriber('pantilt', Vector3, lambda v: tracker.process_position(v))

    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--initial_yaw', type=float, default=175, help='initial yaw for pantilt camera')
    parser.add_argument('--initial_pitch', type=float, default=120, help='initial pitch for pantilt camera')

    track(parser.parse_args())
