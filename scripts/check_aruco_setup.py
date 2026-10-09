#!/usr/bin/env python3
"""Diagnostic fit of decoded marker corners; never publishes robot coordinates or moves the robot."""
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares


def main():
    cfg=json.loads(Path('config/aruco_fresh.json').read_text())
    observation=json.loads(Path('config/aruco_inspection.json').read_text())
    if observation['resolution'] != cfg['intrinsics_resolution']:
        raise ValueError('Inspection resolution differs from intrinsics resolution.')
    markers=cfg['markers']
    pixels=np.vstack([observation['markers'][str(m['id'])] for m in markers])
    centers=[np.asarray(observation['markers'][str(m['id'])]).mean(0) for m in markers]
    base_delta=np.asarray(markers[1]['center_xy_mm'])-markers[0]['center_xy_mm']
    pixel_delta=centers[1]-centers[0]
    similarity=complex(*base_delta)/complex(pixel_delta[0],-pixel_delta[1])
    yaws=[]
    for m in markers:
        corners=np.asarray(observation['markers'][str(m['id'])])
        edge=corners[1]-corners[0]
        yaws.append(np.angle(similarity*complex(edge[0],-edge[1])))
    half=cfg['marker_size_mm']/2
    local=np.array([[-half,half],[half,half],[half,-half],[-half,-half]])
    def objects(angles):
        points=[]
        for m, angle in zip(markers,angles):
            rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
            xy=local@rotation.T+m['center_xy_mm']
            points.extend(np.c_[xy,np.full(4,cfg['marker_plane_z_mm'])])
        return np.asarray(points)
    matrix=np.asarray(cfg['camera_matrix'],dtype=float)
    distortion=np.asarray(cfg['dist_coeffs'],dtype=float)
    ok,rv,tv=cv2.solvePnP(objects(yaws),pixels,matrix,distortion)
    if not ok:raise RuntimeError('Initial pose fit failed.')
    def residual(v):
        projected=cv2.projectPoints(objects(v[6:]),v[:3],v[3:6],matrix,distortion)[0].reshape(-1,2)
        return (projected-pixels).ravel()
    fit=least_squares(residual,np.r_[rv.ravel(),tv.ravel(),yaws],max_nfev=1000)
    if not fit.success:raise RuntimeError('Diagnostic pose fit did not converge.')
    rotation=cv2.Rodrigues(fit.x[:3])[0]
    camera=-rotation.T@fit.x[3:6]
    errors=np.linalg.norm(residual(fit.x).reshape(-1,2),axis=1)
    expected=np.array([*cfg['camera_base_xy_mm'],cfg['marker_plane_z_mm']+cfg['camera_height_above_marker_mm']])
    report=dict(diagnostic_only=True,marker_yaw_estimates_deg={str(m['id']):float((np.rad2deg(a)+180)%360-180) for m,a in zip(markers,fit.x[6:])},
        expected_camera_base_mm=expected.tolist(),fitted_camera_base_mm=camera.tolist(),
        expected_height_above_markers_mm=cfg['camera_height_above_marker_mm'],fitted_height_above_markers_mm=float(camera[2]-cfg['marker_plane_z_mm']),
        camera_xy_difference_mm=float(np.linalg.norm(camera[:2]-expected[:2])),
        camera_z_difference_mm=float(abs(camera[2]-expected[2])),
        reprojection_mean_px=float(errors.mean()),reprojection_max_px=float(errors.max()),
        note='Marker yaw is fitted from the same image, not independently measured. This fit does not validate calibration.')
    report['within_configured_limits']=bool(report['camera_xy_difference_mm']<=cfg['camera_xy_tolerance_mm'] and report['camera_z_difference_mm']<=cfg['camera_height_tolerance_mm'] and errors.max()<=cfg['max_reprojection_error_px'])
    Path('config/aruco_setup_diagnostic.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
