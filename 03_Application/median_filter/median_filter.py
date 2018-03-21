'''
Code written by Dominik Penk, (c) 2018
(Used with permission)

Usage:
python median_filter.py <input_folder> <output_folder> [kernel Size has to be odd]
'''

import sys
import cv2
import os

if __name__ == "__main__":
    folder = sys.argv[1]
    out_folder = sys.argv[2]
    if not os.path.isdir(out_folder):
        os.mkdir(out_folder)
    files = os.listdir(folder)
    filter_size = 3 if len(sys.argv) == 3 else int(sys.argv[3])
    for f in files:
        path = os.path.join(folder, f)
        img = cv2.imread(path)
        img = cv2.medianBlur(img, filter_size)
        cv2.imwrite(os.path.join(out_folder, f), img)
