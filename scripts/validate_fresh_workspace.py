#!/usr/bin/env python3
"""Validate five or more independently measured cube positions via vision service."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from mycobot_interfaces.srv import GetCubeCoords


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('points', type=Path, help='JSON list: {name, color, xy_mm: [x,y]}')
    p.add_argument('--output', type=Path, default=Path('config/fresh_validation.json'))
    p.add_argument('--samples', type=int, default=10)
    p.add_argument('--max-error-mm', type=float, required=True)
    a = p.parse_args()
    points = json.loads(a.points.read_text())
    if len(points)<5 or a.samples<3 or a.max_error_mm<=0: p.error('Use >=5 points, >=3 samples and a positive error limit.')
    for pt in points:
        xy = np.asarray(pt['xy_mm'],dtype=float)
        if xy.shape!=(2,) or not np.isfinite(xy).all(): p.error('Invalid measured XY.')
    rclpy.init(); node = rclpy.create_node('fresh_workspace_validation')
    client = node.create_client(GetCubeCoords,'/cube_coordinates')
    results = []
    try:
        if not client.wait_for_service(timeout_sec=10): raise RuntimeError('Vision service unavailable.')
        for pt in points:
            input(f"Place only the {pt['color']} cube at {pt['name']} {pt['xy_mm']} mm; press Enter: ")
            values = []
            for _ in range(a.samples):
                req=GetCubeCoords.Request(); req.color=pt['color']
                future=client.call_async(req)
                rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                if not future.done() or future.result() is None: raise RuntimeError('Service timed out.')
                coords=np.asarray(future.result().coords,dtype=float)
                if coords.shape!=(6,) or not np.isfinite(coords).all() or np.array_equal(coords,np.ones(6)):
                    raise RuntimeError('Object unavailable; correct detection before repeating validation.')
                values.append(coords[:2]*1000); time.sleep(.15)
            values=np.asarray(values); mean=values.mean(axis=0)
            error=float(np.linalg.norm(mean-np.asarray(pt['xy_mm'])))
            results.append(dict(**pt, detected_xy_mm=mean.tolist(), std_xy_mm=values.std(axis=0).tolist(), error_mm=error))
            print(f'{pt["name"]}: error {error:.2f} mm')
        errors=[r['error_mm'] for r in results]
        report=dict(points=results,mean_error_mm=float(np.mean(errors)),max_error_mm=max(errors),
                    limit_mm=a.max_error_mm,passed=max(errors)<=a.max_error_mm)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
