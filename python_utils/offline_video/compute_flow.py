#!/bin/python

# Script which will use linux shell utilities to compile the video

import argparse
import os
import sys
import cv2
import numpy as np
import shutil

#
# Draw flow on current frame; taken from opencv sample opt_flow.py
#
def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def draw_foe(frame, flow, draw_field=False):
    k = 0
    A_list = []
    b_list = []
    for i in range(10):
        for j in range(90):
            pixel_flow = flow[i,j]
            new_position = (
                int(j + pixel_flow[1]) % 90,
                int(i + pixel_flow[0]) % 10
            )
            int_flow = (pixel_flow[0], pixel_flow[1])
            bk = [(j * int_flow[1]) - (i * int_flow[0])]
            Ak = [int_flow[1], int_flow[0]]
            A_list.append(Ak)
            b_list.append(bk)
            if draw_field and j % 10 == 0 and i % 4 == 0:
                start_point = (j, i)
                end_point = (
                    int(np.ceil(j + pixel_flow[1])),
                    int(np.ceil(i + pixel_flow[0]))
                )
                length = np.sqrt((start_point[0] - end_point[0])**2 + (start_point[1] - end_point[1])**2)
                if(length > 0):
                    #                print(abs(start_point[0] - end_point[0]))
                    print(length)
                    cv2.arrowedLine(frame, start_point, new_position, (0,0,255))
            
    A = np.matrix(A_list)
    b = np.matrix(b_list)

    foe = (A.T * A).I * A.T * b
    foe = (int(np.ceil(foe.item(0))), int(np.ceil(foe.item(1))))
    foe = (foe[0] % 90, foe[1] % 10)

    #Draw FOE
    cv2.circle(frame, (foe[0],foe[1]), 1, (255, 0, 0))
    return frame
            


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "input",
        help="The video input file for processing."
    )

    parser.add_argument(
        "--field",
        action="store_true",
        help="Turn on to draw the optic flow field on each " +
        "frame; every 5th pixel will be shown."
        )
    
    frame_dir = "flow_frames"
    if os.path.isdir(frame_dir):
        print("Removing frame directory");
        shutil.rmtree(frame_dir)

    os.mkdir(frame_dir)

    args = parser.parse_args()
    print("Using OpenCV Version" + cv2.__version__)
    capture = cv2.VideoCapture(args.input)
    os.chdir(frame_dir)
    first = True
    frame_no = 1
    while(True):
        ret, rgb_frame = capture.read()
        if ret == False:
            break

        #
        # If this is the first frame, we can't compute any optical
        # flow vectors, set it to previous, and move to the next frame.
        #
        if first == True:
            rgb_previous_frame = rgb_frame
            previous_frame = cv2.cvtColor(rgb_previous_frame, cv2.COLOR_BGR2GRAY)
            first = False
            continue

        #
        # Now know, previous_frame is defined
        #
        frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)

        flow = cv2.calcOpticalFlowFarneback(
            previous_frame,
            frame,
            None,
            0.5,
            1,
            5,
            3,
            3,
            1.1,
            0
        )

        

        complete = draw_foe(rgb_frame, flow, args.field)
        filename = "frame_%05d.png" % frame_no
        cv2.imwrite(filename, complete)
        frame_no += 1
        #
        # Set current to next previous frame
        #
        previous_frame = frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    #
    # Compile video directly - Change this directory as appropriate
    #
    os.chdir("/home/robert/Uni/project/antbot_rob/python_utils/offline_video/")
    cwd = os.getcwd()
    print(cwd)
    command = "python compile_video.py flow_frames " + cwd
    os.system(command)
        
