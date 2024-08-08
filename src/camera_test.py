#!/usr/bin/env python3

import cv2 
 
if __name__ == "__main__": 
    cap = cv2.VideoCapture(0) 
    while cap.isOpened(): 
        ret, frame = cap.read() 
        if ret: 
            cv2.imshow("camera's frame", frame) 
            c = cv2.waitKey(1) 
            if c == 27: 
                break 
        else: 
            cap.release() 
            cv2.destroyAllWindows()