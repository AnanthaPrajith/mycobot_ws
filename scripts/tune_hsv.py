#!/usr/bin/env python3
"""Tune OpenCV HSV ranges on a raw workspace photograph and save reusable JSON."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

DEFAULT = {
 'red': [[[0,100,100],[15,255,255]], [[170,100,100],[179,255,255]]],
 'yellow': [[[16,100,100],[55,255,255]]],
 'green': [[[56,100,100],[80,255,255]]],
 'blue': [[[81,100,100],[130,255,255]]],
}

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('image', type=Path)
    p.add_argument('--color', choices=DEFAULT, required=True)
    p.add_argument('--interval', type=int, default=0, help='Red: 0 low hue, 1 high hue.')
    p.add_argument('--output', type=Path, default=Path('config/hsv_ranges.json'))
    a = p.parse_args()
    ranges = json.loads(a.output.read_text()) if a.output.exists() else DEFAULT
    if not 0 <= a.interval < len(ranges[a.color]): p.error('Invalid interval.')
    im = cv2.imread(str(a.image))
    if im is None: raise ValueError(f'Cannot read {a.image}')
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    win = 'HSV: s save, q quit'; cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    names = ['H min','S min','V min','H max','S max','V max']
    values = ranges[a.color][a.interval][0]+ranges[a.color][a.interval][1]
    for i, (name, value) in enumerate(zip(names, values)):
        cv2.createTrackbar(name, win, value, 179 if i%3 == 0 else 255, lambda _: None)
    while True:
        v = [cv2.getTrackbarPos(n, win) for n in names]
        intervals = list(ranges[a.color]); intervals[a.interval] = [v[:3],v[3:]]
        raw = np.zeros(hsv.shape[:2],np.uint8)
        for lo, hi in intervals: raw |= cv2.inRange(hsv,tuple(lo),tuple(hi))
        kernel = np.ones((3,3),np.uint8)
        clean = cv2.morphologyEx(cv2.morphologyEx(raw,cv2.MORPH_OPEN,kernel),cv2.MORPH_CLOSE,kernel)
        cv2.imshow(win,cv2.hconcat([im,cv2.cvtColor(raw,cv2.COLOR_GRAY2BGR),cv2.cvtColor(clean,cv2.COLOR_GRAY2BGR)]))
        key = cv2.waitKey(30)&255
        if key == ord('q'): break
        if key == ord('s'):
            if any(lo>hi for lo,hi in zip(v[:3],v[3:])):
                print('Each minimum must be <= its maximum.'); continue
            ranges[a.color][a.interval] = [v[:3],v[3:]]
            a.output.parent.mkdir(parents=True,exist_ok=True)
            a.output.write_text(json.dumps(ranges,indent=2)+'\n')
            cv2.imwrite(str(a.output.with_name(a.color+'_raw_mask.png')),raw)
            cv2.imwrite(str(a.output.with_name(a.color+'_clean_mask.png')),clean)
            print(f'Saved {a.color} interval {a.interval} to {a.output}')
    cv2.destroyAllWindows()

if __name__ == '__main__': main()
