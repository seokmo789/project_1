import sys
import numpy as np
from collections import deque
import time
import cv2  # OpenCV 라이브러리 추가
from queue import PriorityQueue
import copy
from .a_star import *

# gray처리
def convert_to_grayscale_broadcasting(image, weights):
    # 사용자 정의 RGB to Grayscale conversion matrix
    conversion_matrix = np.array(weights)
    
    # 이미지 배열을 (height*width, 3) 형태로 변환
    reshaped_image = image.reshape(-1, 3)
    
    # broadcasting을 사용하여 각 채널의 가중치를 곱해 그레이스케일 값을 계산
    grayscale_values = np.sum(reshaped_image * conversion_matrix, axis=1)
    
    # 그레이스케일 값을 0~255 범위로 클리핑
    grayscale_values = np.clip(grayscale_values, 0, 255).astype(np.uint8)
    
    # 다시 원래 이미지 크기로 변환
    height, width, _ = image.shape
    grayscale_image = grayscale_values.reshape(height, width)
    
    return grayscale_image



def preprocess_image(frame):
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    custom_weights = [0.4, 0.4, 0.9]
    gray = convert_to_grayscale_broadcasting(frame,custom_weights)

    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    return binary

def draw_start_end_points(img, points, mouse_pos=None):
    img_copy = img.copy()
    
    # Draw the start and end points
    for point in points:
        cv2.circle(img_copy, point, 5, (0, 255, 0), -1)
    
    # Draw the mouse position as text if available
    if mouse_pos:
        cv2.putText(img_copy, f"Mouse: {mouse_pos}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    # Show the image
    cv2.imshow("Set Start and End Points", img_copy)

def get_start_end_points(img):
    points = []
    mouse_pos = None

    def mouse_callback(event, x, y, flags, param):
        nonlocal mouse_pos
        if event == cv2.EVENT_MOUSEMOVE:
            mouse_pos = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            draw_start_end_points(img, points, mouse_pos)

    cv2.imshow("Set Start and End Points", img)
    cv2.setMouseCallback("Set Start and End Points", mouse_callback)

    while len(points) < 2:
        draw_start_end_points(img, points, mouse_pos)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
    return points
