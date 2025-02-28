#!/usr/bin/env python3

import rospy
import cv2
import os
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from webots_ros.msg import BoolStamped
from cv_bridge import CvBridge, CvBridgeError
import tiago_pkg.utils as Utils

HOST = os.popen('whoami').read().strip('\n')
PATH = f"/home/{HOST}/catkin_ws/src/webots_ros/config/"

net = cv2.dnn.readNet(f"{PATH}yolov3-tiny.weights", f"{PATH}yolov3-tiny.cfg")
classes = []

with open(f"{PATH}coco.names", "r") as f:
	classes = [line.strip() for line in f.readlines()]

layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))

class ImageConverter(object):

	pedestrian_camera = False
	detect_range_finder = False

	def __init__(self):
		self.node_name = "image_converter"
		rospy.on_shutdown(self.cleanup)
		self.bridge = CvBridge()
		self.enable_camera_range_finder()
		self.image_sub = rospy.Subscriber(Utils.robot_name + "/camera/image", Image, self.camera_callback)
		self.range_image_sub = rospy.Subscriber(Utils.robot_name + "/range_finder/range_image", Image, self.range_finder_callback)

	def camera_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		frame_image = np.array(cv_image, dtype=np.uint8)
		height, width, channels = np.shape(frame_image)
		blob = cv2.dnn.blobFromImage(frame_image, 1 / 255, (416, 416), (0, 0, 0), True, crop=False)
		net.setInput(blob)
		outs = net.forward(output_layers)
		class_ids = []
		confidences = []
		boxes = []

		for out in outs:
			for detection in out:
				scores = detection[5:]
				class_id = np.argmax(scores)
				confidence = scores[class_id]
				if confidence > 0.7 and class_id == 0: # confidance pedestrian
					center_x = int(detection[0] * width)
					center_y = int(detection[1] * height)
					w = int(detection[2] * width)
					h = int(detection[3] * height)
					x = int(center_x - w / 2)
					y = int(center_y - h / 2)
					boxes.append([x, y, w, h])
					confidences.append(float(confidence))
					class_ids.append(class_id)

		indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
		font = cv2.FONT_HERSHEY_PLAIN

		if len(boxes) > 0:
			for i in range(len(boxes)):
				if i in indexes:
					x, y, w, h = boxes[i]
					label = str(classes[class_ids[i]])
					color = colors[i]
					cv2.rectangle(frame_image, (x, y), (x + w, y + h), color, 2)
					cv2.putText(frame_image, f"{label}\t{round(confidences[i], 2)}", (x, y + 30), font, 3, color, 3)
					self.pedestrian_camera = True
		else:
			self.pedestrian_camera = False

		## Display image
		cv2.imshow(self.node_name, frame_image)
		### Proces any keyboard commands
		self.keystroke = cv2.waitKey(3)
		if 32 <= self.keystroke and self.keystroke < 128:
			cc = chr(self.keystroke).lower()
			if cc == 'q':
				# The user has press the q key, so exit
				rospy.signal_shutdown("User hit q key to quit.")
	
	def range_finder_callback(self, data):
		avg = sum(data.data)/len(data.data)
		if avg < 30 or avg > 35:
			self.detect_range_finder = True
		else:
			self.detect_range_finder = False

	def get_detection(self):
		return self.pedestrian_camera and self.detect_range_finder
	
	def cleanup(self):
		cv2.destroyAllWindows()

	def enable_camera_range_finder(self):
		Utils.call_service('camera', 'enable', 1)
		Utils.call_service('range_finder', 'enable', 1)

def main():
	try:
		rospy.init_node("image_converter", anonymous=True)
		r = rospy.Rate(5)
		ic = ImageConverter()
		pub = rospy.Publisher("pedestrian_detect", BoolStamped, queue_size=1)
		
		while not rospy.is_shutdown():
			is_detect = BoolStamped()
			is_detect.header.stamp = rospy.Time.now()
			is_detect.header.frame_id = "pedestrian_detect"
			is_detect.data = ic.get_detection()
			pub.publish(is_detect)
			r.sleep()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()