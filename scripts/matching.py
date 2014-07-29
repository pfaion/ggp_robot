#!/usr/bin/env python
import sys
import time
import numpy as np
import scipy as sp
import scipy.misc
import cv2
import roslib
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

class image_feature:

  def __init__(self):
    """Initialize ros-subscriber and pre-process reference-image."""

    # set up subscriber
    self.bridge = CvBridge()
    self.subscriber = rospy.Subscriber("/camera/rgb/image_raw", sensor_msgs.msg.Image, self.callback,  queue_size = 1)

    # load reference image
    refImg_path = "/home/pfaion/catkin_ws/src/ggp_robot/src/board_low2.jpg"
    self.refImg = cv2.imread(refImg_path, cv2.CV_LOAD_IMAGE_COLOR)
    
    # compute features of reference image
    (self.refGray,
     self.refRegions,
     self.refPoints,
     self.refDes) = self.computeFeatures(self.refImg, more=True)
    print len(self.refPoints)

  def computeFeatures(self, img, more=False):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    mser = cv2.MSER()
    if more:
      mser.setDouble('maxVariation', 5)
      mser.setDouble('delta', 2)
      mser.setDouble('minDiversity', 0.05)
      mser.setDouble('maxArea', 50000)
    regions = mser.detect(gray, None)
    hulls = self.getHulls(regions)

    contours = np.array([r.reshape(r.shape[0], 1, -1) for r in regions])

    invalidIdx = []
    centroids = []
    for i, cnt in enumerate(contours):
      if len(cnt) < 5:
        invalidIdx.append(i)
        continue
      rect = cv2.fitEllipse(cnt)
      width = rect[1][0]
      height = rect[1][1]
      size = np.sqrt(width*height)
      x = rect[0][0]
      y = rect[0][1]
      angleRad = self.maxOrientationFast(gray,cnt)
      angleDeg = np.degrees(angleRad)
      centroids.append(cv2.KeyPoint(x, y, size, angleDeg))
      
    for i in sorted(invalidIdx, reverse=True):
      del contours[i]

    sift = cv2.SIFT()
    _,descriptors = sift.compute(gray,centroids)

    return (gray, contours, centroids, descriptors)


  def visualize(self, img1, img2, fp1, fp2, reg1, reg2, matches):
    if 0:
      self.drawHulls(img1, [self.getHulls(reg1)[0]])
      self.drawHulls(img2, self.getHulls(reg2))
      img1 = self.drawFPs(img1, [fp1[0]])
      img2 = self.drawFPs(img2, fp2)
    else:
      self.drawHulls(img1, self.getHulls(reg1))
      self.drawHulls(img2, self.getHulls(reg2))
      img1 = self.drawFPs(img1, fp1)
      img2 = self.drawFPs(img2, fp2)

    if 0:
      for reg in reg1:
        r = cv2.boundingRect(reg)
        x1 = max(0, r[0])
        x2 = min(x1 + r[2], img1.shape[1])
        y1 = max(0, r[1])
        y2 = min(y1 + r[3], img1.shape[0])
        cv2.rectangle(img1, (x1, y1), (x2, y2), self.randColor())

   

    h1,w1 = img1.shape[:2]
    h2,w2 = img2.shape[:2]
    view = sp.zeros((max(h1,h2), w1 + w2, 3), sp.uint8)
    view[:h1, :w1, :] = img1
    view[:h2, w1:, :] = img2

    if matches != None:
      for m in matches:
        cv2.line(view, (int(fp1[m.queryIdx].pt[0]), int(fp1[m.queryIdx].pt[1])), (int(fp2[m.trainIdx].pt[0] + w1), int(fp2[m.trainIdx].pt[1])), self.randColor())
    return view
   
  def randColor(self):
    r = np.random.randint(256, size=1)[0]
    g = np.random.randint(256, size=1)[0]
    b = np.random.randint(256, size=1)[0]
    return (r, g, b)

  def getHulls(self, regions):
    return [cv2.convexHull(r.reshape(-1, 1, 2)) for r in regions]

  def drawHulls(self, img, hulls):
    for hull in hulls:
      cv2.polylines(img, [hull], 1, self.randColor())

  def drawFPs(self, img, points):
    img = cv2.drawKeypoints(img, points, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    for fp in points:
      x,y = fp.pt
      cv2.circle(img, (int(x), int(y)), 1, (0,0,255), -1)
    return img

  def maxOrientationFast(self, img, reg):
    r = cv2.boundingRect(reg)
    x1 = max(0, r[0])
    x2 = min(x1 + r[2], img.shape[1])
    y1 = max(0, r[1])
    y2 = min(y1 + r[3], img.shape[0])
    patch = img[y1:y2,x1:x2].copy()
    sobelx = cv2.Sobel(patch,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(patch,cv2.CV_64F,0,1)
    ori = cv2.phase(sobelx, sobely)
    mag = cv2.magnitude(sobelx, sobely)
    maxRad = ori.flatten()[np.argmax(mag)]
    return maxRad


  def maxOrientation(self, img, reg):
    r = cv2.boundingRect(reg)
    x1 = min(0, r[0])
    x2 = max(x1 + r[2], img.shape[0])
    y1 = min(0, r[1])
    y2 = max(y1 + r[3], img.shape[1])
    patch = img[x1:x2,y1:y2].copy()
    sobelx = cv2.Sobel(patch,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(patch,cv2.CV_64F,0,1)
    ori = cv2.phase(sobelx, sobely)
    mag = cv2.magnitude(sobelx, sobely)
    n = 36
    hist = [0]*n
    it = np.nditer(ori, flags=['multi_index'])
    while not it.finished:
      binIdx = int(round((n-1)/(2*np.pi)*it[0]))
      hist[binIdx] =+ mag[it.multi_index[0]][it.multi_index[1]]
      it.iternext()
    maxRad = (n-1)/(2*np.pi)/max(hist)
    print maxRad
    return maxRad

  def callback(self, ros_data):
    """ Get image from xtion, perform feature matching and pose estimation."""
    
    # load the image from ROS
    try:
      camImg = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
    except CvBridgeError, e:
      print e
    
    # compute features of camera image
    (camGray, camRegions, camPoints, camDes) = self.computeFeatures(camImg)
    
    

    matchImg = self.visualize(camImg, self.refImg,
                        camPoints, self.refPoints,
                        camRegions, self.refRegions,
                        None)

    matcher = Matcher() 
    matcher.match(camGray, self.refGray,
                  camPoints, self.refPoints,
                  camRegions, self.refRegions,
                  camDes, self.refDes)

    cv2.imshow('cv_img', matchImg)
    cv2.waitKey(2)


  
class Matcher:
  
  def __init__(self):
    pass

  def match(self, img1, img2, fp1, fp2, reg1, reg2, des1, des2):
    matches = []
    for i1, v_i1 in enumerate(des1):
      for i2, v_i2 in enumerate(des2):
        if len(v_i1) != len(v_i2):
          raise Exception("uncompatible feature vectors!")
        dist = np.linalg.norm(v_i1 - v_i2)
        matches.append(cv2.DMatch(i1, i2, dist))
        break
      break





def main(args):
  '''Initializes and cleanup ros node'''
  ic = image_feature()
  rospy.init_node('image_feature', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down ROS Image feature detector module"
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
