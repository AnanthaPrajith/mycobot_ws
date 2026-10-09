#!/usr/bin/env python3
"""Capture raw chessboard photos from the PC camera: s saves, q quits."""
import argparse
from datetime import datetime
from pathlib import Path
import cv2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--camera', default='/dev/video0')
    p.add_argument('--columns', type=int, required=True)
    p.add_argument('--rows', type=int, required=True)
    p.add_argument('--width', type=int, default=640)
    p.add_argument('--height', type=int, default=480)
    p.add_argument('--output', type=Path, default=Path('calibration_images'))
    a = p.parse_args()
    if min(a.columns,a.rows)<3 or min(a.width,a.height)<=0:
        p.error('Use >=3 inner corners per axis and a positive resolution.')
    cap = cv2.VideoCapture(int(a.camera) if a.camera.isdigit() else a.camera)
    try:
        if not cap.isOpened(): p.error(f'Cannot open {a.camera}; check the device and stop other camera nodes.')
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT,a.height)
        a.output.mkdir(parents=True,exist_ok=True)
        count=0
        print('Move the board across the image with varied tilts/distances. s saves detected boards; q quits.')
        while True:
            ok, frame=cap.read()
            if not ok: raise RuntimeError('Camera frame capture failed.')
            h,w=frame.shape[:2]
            if (w,h)!=(a.width,a.height):
                raise RuntimeError(f'Camera returned {w}x{h}; requested {a.width}x{a.height}. Choose a supported resolution.')
            found,corners=cv2.findChessboardCorners(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),(a.columns,a.rows))
            display=frame.copy()
            if found: cv2.drawChessboardCorners(display,(a.columns,a.rows),corners,found)
            cv2.putText(display,f'{w}x{h} board={found} saved={count} | s save, q quit',(8,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0) if found else (0,0,255),1)
            cv2.imshow('Calibration capture',display)
            key=cv2.waitKey(20)&255
            if key==ord('q'): break
            if key==ord('s'):
                if not found:
                    print('No full chessboard detected; check inner corner counts and visibility.'); continue
                path=a.output/('chessboard_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.png')
                if not cv2.imwrite(str(path),frame): raise RuntimeError(f'Cannot save {path}')
                count+=1; print(path)
    finally:
        cap.release(); cv2.destroyAllWindows()

if __name__=='__main__': main()
