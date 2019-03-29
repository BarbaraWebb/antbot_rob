#!/bin/python

# Script which will use linux shell utilities to compile the video

import argparse
import os
import sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "frame_directory",
        help="The directory containing the image frames you wish to render " +
        "into a video."
    )

    parser.add_argument(
        "output_directory",
        help="The output directory for the mp4 file."
        )

    args = parser.parse_args()

    os.chdir(args.frame_directory)
    print("Switched to : " + os.getcwd())
    print("Using FFMPEG to render the video.")
    output = args.output_directory + "/out.mp4"
    os.system("ffmpeg -r 8 -f image2 -s 900x100 -i frame_%05d.png -vcodec "+
              "libx264 -crf 25  -pix_fmt yuv420p " + output)
