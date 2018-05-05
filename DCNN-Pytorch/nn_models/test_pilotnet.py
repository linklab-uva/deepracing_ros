import cv2
import numpy as np
import nn_models
import data_loading.image_loading as il
import nn_models.Models as models
import data_loading.data_loaders as loaders
import numpy.random
import torch, random
import torch.nn as nn 
import torch.optim as optim
from tqdm import tqdm as tqdm
import pickle
from datetime import datetime
import os
import string
import argparse
from random import randint
from datetime import datetime
import matplotlib.pyplot as plt
import imutils.annotation_utils
from scipy import stats
from data_loading.image_loading import load_image
def main():
    parser = argparse.ArgumentParser(description="Deepf1 playground")
    parser.add_argument("--x", type=int, default=25,  help="X coordinate for where to put the center of the steering wheel")
    parser.add_argument("--y", type=int, default=25,  help="Y coordinate for where to put the center of the steering wheel")
    parser.add_argument("--wheelrows", type=int, default=50,  help="Number of rows to resize the steering wheel to")
    parser.add_argument("--wheelcols", type=int, default=50,  help="Number of columns to resize the steering wheel to")
    parser.add_argument("--max_angle", type=float, default=180.0,\
          help="Maximum angle that the scaled annotations represent")
    parser.add_argument("--output_folder", type=str, default='prediction_images',\
          help="Output video file")
    parser.add_argument("--output_video", type=str, default='annotated_video.avi',\
          help="Output video file")
    parser.add_argument("--model_file", type=str, required=True,  help="Model weights to load from file")
    parser.add_argument("--root_dir", type=str, required=True, help="Root dir of the F1 validation set to use")
    parser.add_argument("--annotation_file", type=str, required=True, help="Annotation file to use")
    args = parser.parse_args()
    
    network = models.PilotNet()
    network.float()
    network.cuda()
    state_dict = torch.load(args.model_file)
    network.load_state_dict(state_dict)
    output_video = args.output_video
    wheelrows = args.wheelrows
    wheelcols = args.wheelcols
    x = int(args.x-wheelrows/2)
    y = int(args.y-wheelcols/2)
    max_angle = args.max_angle
    wheel = cv2.imread('steering_wheel.png', cv2.IMREAD_UNCHANGED)  
    wheel =  cv2.resize(wheel, (wheelrows, wheelcols)) 
    input_folder = os.path.join(args.root_dir,'raw_images')
    if(not os.path.isdir(args.output_folder)):
        os.makedirs(args.output_folder)
    annotations_path = os.path.join(args.root_dir,args.annotation_file)
    annotations_file = open(annotations_path,'r')
    annotations = annotations_file.readlines()
    predictions = []
    ground_truths = []
    diffs = []
    filename, _, anglestr, _, _ = annotations[0].split(",")
    img_path = os.path.join(input_folder,filename)
    background = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
    size = background.shape
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    output_vid = os.path.join(args.output_folder,output_video)
    videoout = cv2.VideoWriter(output_vid ,fourcc, 60.0, (size[1], size[0]),True)
    network.eval()
    for idx, annotation in tqdm(enumerate(annotations)):
        filename, _, anglestr, _, _ = annotation.split(",")
        img_path = os.path.join(input_folder,filename)
        background = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
        im = load_image(img_path,size=(66,200),scale_factor=2.55, use_float32 = True)
        input = np.random.rand(1,3,66,200).astype(np.float32)
        input[0] = im
        input = np.divide(input, SCALE_FACTOR)
        tensor = torch.from_numpy(input).cuda()
        pred = network(tensor)
        angle = pred.item()/100.0
        ground_truth = float(anglestr.replace("\n",""))
        diffs.append(ground_truth-angle)
        scaled_ground_truth = max_angle * ground_truth
        scaled_angle = max_angle * angle
       # print("Ground Truth: %f. Prediction: %f.\n" %(scaled_ground_truth, scaled_angle))
        M = cv2.getRotationMatrix2D((wheelrows/2,wheelcols/2),scaled_angle,1)
        wheel_rotated = cv2.warpAffine(wheel,M,(wheelrows,wheelcols))
        overlayed = imutils.annotation_utils.overlay_image(background,wheel_rotated,x,y)
        name, _ = filename.split(".")
        _,_,img_num_str = name.split("_")
        img_num_str = img_num_str.replace("\n","")
        output_path = os.path.join(args.output_folder,'overlayed_image_' + img_num_str + ".jpg")
        cv2.imwrite(output_path,overlayed)
        videoout.write(overlayed)
    binz = 100
    res = stats.cumfreq(diffs, numbins=binz)
    x = res.lowerlimit + np.linspace(0, res.binsize*res.cumcount.size, res.cumcount.size)
    fig = plt.figure(figsize=(10, 4))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)
    ax1.hist(diffs, bins=binz)
    ax1.set_title('Histogram')
    ax2.bar(x, res.cumcount, width=res.binsize)
    ax2.set_title('Cumulative histogram')
    ax2.set_xlim([x.min(), x.max()])
    plt.show()
if __name__ == '__main__':
    main()
