import random
from itertools import count
from pyexpat import model
import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import io
from io import StringIO 
from matplotlib import pyplot as plt
import time
from PIL import Image
import base64,cv2
import pyshine as ps
from flask_cors import CORS,cross_origin
import imutils
import dlib
from tensorflow import keras
from engineio.payload import Payload
app = Flask(__name__)
app.config['DEBUG'] = True
cors = CORS(app,resources={r"/api/*":{"origins":"*"}})
socketio = SocketIO(app, cors_allowed_origins="*")
import os
import glob
import torch
import utils
import argparse

import cv2
import image_filtering
import algorithm
import os
from pathlib import Path

from imutils.video import VideoStream
from midas.model_loader import default_models, load_model

first_execution = True



@app.route('/', methods=['POST', 'GET'])
def index():
    return render_template('index.html')

def readb64(base64_string):
    idx = base64_string.find('base64,')
    base64_string  = base64_string[idx+7:]

    sbuf = io.BytesIO()

    sbuf.write(base64.b64decode(base64_string, ' /'))
    pimg = Image.open(sbuf)


    return cv2.cvtColor(np.array(pimg), cv2.COLOR_RGB2BGR)

def process(device, model, model_type, image, input_size, target_size, optimize, use_camera):
    """
    Run the inference and interpolate.

    Args:
        device (torch.device): the torch device used
        model: the model used for inference
        model_type: the type of the model
        image: the image fed into the neural network
        input_size: the size (width, height) of the neural network input (for OpenVINO)
        target_size: the size (width, height) the neural network output is interpolated to
        optimize: optimize the model to half-floats on CUDA?
        use_camera: is the camera used?

    Returns:
        the prediction
    """
    global first_execution

    if "openvino" in model_type:
        if first_execution or not use_camera:
            print(f"    Input resized to {input_size[0]}x{input_size[1]} before entering the encoder")
            first_execution = False

        sample = [np.reshape(image, (1, 3, *input_size))]
        prediction = model(sample)[model.output(0)][0]
        prediction = cv2.resize(prediction, dsize=target_size,
                                interpolation=cv2.INTER_CUBIC)
    else:
        sample = torch.from_numpy(image).to(device).unsqueeze(0)

        if optimize and device == torch.device("cuda"):
            if first_execution:
                print("  Optimization to half-floats activated. Use with caution, because models like Swin require\n"
                      "  float precision to work properly and may yield non-finite depth values to some extent for\n"
                      "  half-floats.")
            sample = sample.to(memory_format=torch.channels_last)
            sample = sample.half()

        if first_execution or not use_camera:
            height, width = sample.shape[2:]
            print(f"    Input resized to {width}x{height} before entering the encoder")
            first_execution = False

        prediction = model.forward(sample)
        prediction = (
            torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=target_size[::-1],
                mode="bicubic",
                align_corners=False,
            )
            .squeeze()
            .cpu()
            .numpy()
        )

    return prediction


def create_side_by_side(image, depth, grayscale):
    """
    Take an RGB image and depth map and place them side by side. This includes a proper normalization of the depth map
    for better visibility.

    Args:
        image: the RGB image
        depth: the depth map
        grayscale: use a grayscale colormap?

    Returns:
        the image and depth map place side by side
    """
    depth_min = depth.min()
    depth_max = depth.max()
    normalized_depth = 255 * (depth - depth_min) / (depth_max - depth_min)
    normalized_depth *= 3

    right_side = np.repeat(np.expand_dims(normalized_depth, 2), 3, axis=2) / 3
    if not grayscale:
        right_side = cv2.applyColorMap(np.uint8(right_side), cv2.COLORMAP_INFERNO)

    if image is None:
        return right_side
    else:
        return np.concatenate((image, right_side), axis=1)


def run(frame, input_path, output_path, model_path, model_type="dpt_beit_large_512", optimize=False, side=False, height=None,
        square=False, grayscale=False):
    """Run MonoDepthNN to compute depth maps.

    Args:
        input_path (str): path to input folder
        output_path (str): path to output folder
        model_path (str): path to saved model
        model_type (str): the model type
        optimize (bool): optimize the model to half-floats on CUDA?
        side (bool): RGB and depth side by side in output images?
        height (int): inference encoder image height
        square (bool): resize to a square resolution?
        grayscale (bool): use a grayscale colormap?
    """
    print("Initialize")

    # select device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Device: %s" % device)

    model, transform, net_w, net_h = load_model(device, model_path, model_type, optimize, height, square)

    # get input
    if input_path is not None:
        image_names = glob.glob(os.path.join(input_path, "*"))
        num_images = len(image_names)
    else:
        print("No input path specified. Grabbing images from camera.")

    # create output folder
    if output_path is not None:
        os.makedirs(output_path, exist_ok=True)

    print("Start processing")

    if input_path is not None:
        if output_path is None:
            print("Warning: No output path specified. Images will be processed but not shown or stored anywhere.")
        for index, image_name in enumerate(image_names):

            print("  Processing {} ({}/{})".format(image_name, index + 1, num_images))

            # input
            original_image_rgb = utils.read_image(image_name)  # in [0, 1]
            image = transform({"image": original_image_rgb})["image"]

            # compute
            with torch.no_grad():
                prediction = process(device, model, model_type, image, (net_w, net_h), original_image_rgb.shape[1::-1],
                                     optimize, False)

            # output
            if output_path is not None:
                filename = os.path.join(
                    output_path, os.path.splitext(os.path.basename(image_name))[0] + '-' + model_type
                )
                if not side:
                    utils.write_depth(filename, prediction, grayscale, bits=2)
                else:
                    try:
                        original_image_bgr = np.flip(original_image_rgb, 2)
                        content = create_side_by_side(original_image_bgr*255, prediction, grayscale)
                        cv2.imwrite(filename + ".png", content)
                    except Exception as e:
                        pass
                utils.write_pfm(filename + ".pfm", prediction.astype(np.float32))

    else:
        with torch.no_grad():
            # cap = cv2.VideoCapture('input\\clg_video.mp4')
            # fps = int(cap.get(cv2.CAP_PROP_FPS))
            # frame_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            # time_start = time.time()
            # frame_index = 0 
            # # fps = 1
            # fourcc = cv2.VideoWriter_fourcc(*'H264')  # Use H264 codec
            # out = cv2.VideoWriter('output.mp4', fourcc, fps, frame_size, isColor=True)
            # while cap.isOpened(): 
                # frame = "file name yaha daalo hehe"
                if frame is not None:
  
                    # content = cv2.GaussianBlur(frame, (5, 5), 0)  
                    original_image_rgb = np.flip(frame, 2)  # in [0, 255] (flip required to get RGB)
                    image = transform({"image": original_image_rgb/255})["image"]

                    prediction = process(device, model, model_type, image, (net_w, net_h),
                                         original_image_rgb.shape[1::-1], optimize, True)

                    original_image_bgr = np.flip(original_image_rgb, 2) if side else None
                    content = create_side_by_side(original_image_bgr, prediction, grayscale)
###################################################################################################################
                    # content = cv2.GaussianBlur(content, (5, 5), 0)  

                    content = image_filtering.apply_laplacian(content)

                    #Thresholding
                    content, h, w, _ = image_filtering.apply_thresholding(content)

                    # Dilation 
                    content = image_filtering.apply_dilation(content, h, w)

                    # Algo
                    img2 = content.copy()
                    center, sightly_left, sightly_right, left, right, img2 = algorithm.apply_algorithm(content, img2, h, w)

                    # Decision
                    decision = algorithm.apply_left_right(center, sightly_left, sightly_right, left, right)
                    # cv2.imshow('MiDaS Depth Estimation - Press Escape to close window ', content)
                   
                    var = random.random()
                    filename = "content" + str(var) + ".png"
                    cv2.imwrite(filename , img2)
                    return decision
# ###################################################################################################################                    

                    # out.write(content)

                    # alpha = 0.1
                    # if time.time()-time_start > 0:
                    #     fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
                    #     time_start = time.time()
                    # print(f"\rFPS: {round(fps,2)}", end="")

                    # # if cv2.waitKey(1) == 27:  # Escape key
                    # #     break

                    # frame_index += 1   
                    # # return decision  
        print()         

    print("Finished")
@socketio.on('image')
def image(data_image):
    # Process the image frame
    # name = str("frame.jpg")
    frame = (readb64(data_image))
    input_path = None
    output_path = None
    model_type = "dpt_swin2_tiny_256"
    model_weights = None
    if model_weights is None:
        model_weights = default_models[model_type]

    # set torch options
    torch.backends.cudnn.enabled = True
    torch.backends.cudnn.benchmark = True
    # compute depth maps
    
    sendOp = run(frame, input_path, output_path, model_weights, model_type)
    print(f"Namaste dosto me hu OP: {sendOp}")
    # cv2.imshow("frame",frame)
    # cv2.waitKey(0)
    # cv2.imwrite(name,frame)
    # imgencode = cv2.imencode('.jpg', frame)[1]

    # base64 encode
    # stringData = base64.b64encode(imgencode).decode('utf-8')
    # b64_src = 'data:image/jpg;base64,'
    # stringData = b64_src + stringData

    # emit the frame back
    emit('response_back', {'data': sendOp})





if __name__ == '__main__':
    socketio.run(app, host='127.0.0.1', port = '5000')
    # 127.0.0.1