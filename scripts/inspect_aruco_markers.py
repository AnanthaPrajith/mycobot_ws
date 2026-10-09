#!/usr/bin/env python3
"""Inspect both markers from the existing ROS camera publisher, without robot motion."""
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    rclpy.init()
    node=Node('inspect_aruco_markers')
    bridge=CvBridge()
    cfg=json.loads(Path('config/aruco_fresh.json').read_text())
    dictionary=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,cfg['dictionary']))
    params=cv2.aruco.DetectorParameters() if hasattr(cv2.aruco,'ArucoDetector') else cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    detector=cv2.aruco.ArucoDetector(dictionary,params) if hasattr(cv2.aruco,'ArucoDetector') else None
    result={}
    def callback(msg):
        frame=bridge.imgmsg_to_cv2(msg,'bgr8')
        corners,ids,_=detector.detectMarkers(frame) if detector else cv2.aruco.detectMarkers(frame,dictionary,parameters=params)
        if ids is None:return
        mapping={int(i):c.reshape(4,2) for i,c in zip(ids.ravel(),corners)}
        if not all(m['id'] in mapping for m in cfg['markers']):return
        Path('calibration_images').mkdir(exist_ok=True)
        cv2.imwrite('calibration_images/aruco_raw.png',frame)
        cv2.aruco.drawDetectedMarkers(frame,corners,ids)
        for marker in cfg['markers']:
            i=marker['id'];c=mapping[i]
            cv2.arrowedLine(frame,tuple(np.round(c[0]).astype(int)),tuple(np.round(c[1]).astype(int)),(0,0,255),2)
        cv2.imwrite('calibration_images/aruco_inspected.png',frame)
        result.update(resolution=[frame.shape[1],frame.shape[0]],markers={str(i):c.tolist() for i,c in mapping.items()})
    sub=node.create_subscription(Image,'camera/image',callback,1)
    try:
        deadline=time.monotonic()+20
        while not result and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.5)
        if not result:raise RuntimeError('Both configured markers were not detected within 20 seconds; check publisher, domain and visibility.')
        Path('config/aruco_inspection.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        print('Saved calibration_images/aruco_inspected.png and config/aruco_inspection.json')
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
