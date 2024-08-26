from flask import Blueprint, render_template, Response, request, send_file, jsonify
import cv2
import numpy as np
import io
import sys
import argparse
import imutils
import time
import math
import mmap
import os
import pickle
import threading
import atexit

from collections import deque
from imutils.video import VideoStream

from config import *
from .a_star import *
from .drawer_func_utils import *


bp = Blueprint('camera', __name__, url_prefix='/camera')
#-----------------------------------------전역변수
last_frame = None  # 전역 변수로 최근 프레임을 저장
trans_end_red_position=None
green_to_red_angle=None
not_trance_end_red_position=None
not_trance_center_green_position=None
neww=None
newh=None
g_captrue_memory_start=0
pts = deque(maxlen=64)

#-------------------------------------------------------------------
def delete_shared_memory():
    filepath = "shared_memory.dat"
    if os.path.exists(filepath):
        os.remove(filepath)
        print("Shared memory file deleted.")
    else:
        print("Shared memory file does not exist.")
atexit.register(delete_shared_memory)

def gen_frames():
    global last_frame
    while True:
        if last_frame is not None:
            # 프레임을 JPEG 형식으로 인코딩하여 바이트 스트림으로 변환
            ret, buffer = cv2.imencode('.jpg', last_frame)
            frame_bytes = buffer.tobytes()

            # 바이트 스트림을 yield하여 generator로 반환
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@bp.route('/video_feed', methods=['POST'])
def video_feed():
    global last_frame
    if 'video' not in request.files:
        return "No video part in the request", 400

    file = request.files['video']
    img_array = np.frombuffer(file.read(), np.uint8)
    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    if frame is None:
        return "Failed to decode image", 400

    # 받은 프레임을 전역 변수에 저장
    last_frame = frame

    return "Frame received", 200

@bp.route('/display')
def display():
    
    return render_template('cameradisplay.html')

@bp.route('/stream')
def stream():
    # Response 객체를 사용하여 generator를 mimetype으로 반환
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@bp.route('/stream2')
def stream2():
    return Response(gen_frames2(), mimetype='multipart/x-mixed-replace; boundary=frame')

def is_valid_contour(contour):
    # 사각형 비율을 기준으로 필터링
    rect = cv2.boundingRect(contour)
    aspect_ratio = float(rect[2]) / rect[3]
    return 0.5 < aspect_ratio < 2.0  # 가로 세로 비율 필터링

pts_red = deque(maxlen=64)
pts_green = deque(maxlen=64)

def gen_frames2():
    global trans_end_red_position, green_to_red_angle, neww, newh, not_trance_end_red_position, not_trance_center_green_position, g_captrue_memory_start
    while True:
        if last_frame is not None:
            # 원본 프레임을 복사하여 변형
            frame = last_frame.copy()

            frame = imutils.resize(frame, width=600)
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)

            # 프레임을 HSV 색상 공간으로 변환
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # 빨간색 공을 검출하기 위한 색상 범위 설정
            red_lower1 = (0, 120, 70)
            red_upper1 = (10, 255, 255)
            red_lower2 = (170, 120, 70)
            red_upper2 = (180, 255, 255)

            # 초록색 공을 검출하기 위한 두 개의 색상 범위 설정
            green_lower1 = (35, 100, 100)
            green_upper1 = (65, 255, 255)
            green_lower2 = (65, 100, 100)
            green_upper2 = (85, 255, 255)

            # 빨간색 공 검출
            red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
            red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
            red_mask = red_mask1 | red_mask2

            # 초록색 공 검출
            green_mask1 = cv2.inRange(hsv, green_lower1, green_upper1)
            green_mask2 = cv2.inRange(hsv, green_lower2, green_upper2)
            green_mask = green_mask1 | green_mask2

            # 두 마스크를 결합
            combined_mask = red_mask | green_mask

            # 노이즈 제거를 위해 마스크를 침식하고 팽창
            combined_mask = cv2.erode(combined_mask, None, iterations=2)  # 침식
            combined_mask = cv2.dilate(combined_mask, None, iterations=2)  # 팽창

            # 마스크에서 윤곽선을 찾아서 공을 검출
            cnts_red = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_red = imutils.grab_contours(cnts_red)
            end_red = None

            cnts_green = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts_green = imutils.grab_contours(cnts_green)
            center_green = None

            if len(cnts_red) > 0:
                # 가장 큰 윤곽선을 찾아 최소 enclosing circle과 중심을 계산
                c_red = max(cnts_red, key=cv2.contourArea)
                M_red = cv2.moments(c_red)

                if M_red["m00"] != 0:
                    ((x_red, y_red), radius_red) = cv2.minEnclosingCircle(c_red)
                    end_red = int(M_red["m10"] / M_red["m00"]), int(M_red["m01"] / M_red["m00"])

                    # 반지름이 최소 크기를 넘는 경우
                    if radius_red > 10:
                        cv2.circle(frame, (int(x_red), int(y_red)), int(radius_red), (0, 0, 255), 2)
                        cv2.circle(frame, end_red, 5, (0, 0, 255), -1)
                else:
                    end_red = None  # 모멘트가 0이면 중심을 계산하지 않음

            if len(cnts_green) > 0:
                c_green = max(cnts_green, key=cv2.contourArea)
                M_green = cv2.moments(c_green)

                if M_green["m00"] != 0:
                    ((x_green, y_green), radius_green) = cv2.minEnclosingCircle(c_green)
                    center_green = int(M_green["m10"] / M_green["m00"]), int(M_green["m01"] / M_green["m00"])

                    if radius_green > 10:
                        cv2.circle(frame, (int(x_green), int(y_green)), int(radius_green), (0, 255, 0), 2)
                        cv2.circle(frame, center_green, 5, (0, 255, 0), -1)
                else:
                    center_green = None  # 모멘트가 0이면 중심을 계산하지 않음

            # 빨간점에서 초록점까지의 각도를 계산
            if end_red is not None and center_green is not None:
                vector = np.array(end_red) - np.array(center_green)
                angle = np.degrees(np.arctan2(vector[1], vector[0]))
                if angle < 0:
                    angle += 360
                angle_text = f"green to red angle : {int(angle)} degrees"
                cv2.putText(frame, angle_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            else:
                angle = None  # 각도 계산 불가능한 경우

            # 점의 리스트에 중심을 추가
            pts_red.appendleft(end_red)
            pts_green.appendleft(center_green)

            # 추적된 점들을 화면에 표시
            for i in range(1, len(pts_red)):
                if pts_red[i - 1] is None or pts_red[i] is None:
                    continue

                # 선의 두께를 계산하고 선을 그립니다
                thickness_red = int(np.sqrt(64 / float(i + 1)) * 2.5)
                cv2.line(frame, pts_red[i - 1], pts_red[i], (0, 0, 255), thickness_red)

            for i in range(1, len(pts_green)):
                if pts_green[i - 1] is None or pts_green[i] is None:
                    continue

                # 선의 두께를 계산하고 선을 그립니다
                thickness_green = int(np.sqrt(64 / float(i + 1)) * 2.5)
                cv2.line(frame, pts_green[i - 1], pts_green[i], (0, 255, 0), thickness_green)

            if end_red is not None:
                transformed_text_red = f"Red Ball Position: {end_red}"
                cv2.putText(frame, transformed_text_red, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

            if center_green is not None:
                transformed_text_green = f"Green Ball Position: {center_green}"
                cv2.putText(frame, transformed_text_green, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

            # ball_convert_a_star 함수 호출 전에 None 체크
            if end_red is not None:
                trans_end_red_position, newh, neww = ball_convert_a_star(frame, end_red)
                not_trance_end_red_position = (int(end_red[0]), int(end_red[1]))
            else:
                trans_end_red_position, not_trance_end_red_position = None, None

            if center_green is not None:
                not_trance_center_green_position = (int(center_green[0]), int(center_green[1]))
            else:
                not_trance_center_green_position = None

            green_to_red_angle = angle

            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()

            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# 이부분은 capture부분 구형해야함 지금은 없어도 동작됨
@bp.route('/capture_image', methods=['GET'])
def capture_image():
    global last_frame,g_captrue_memory_start
    g_captrue_memory_start=1 #쉐어드 메모리 스타트
    if last_frame is None:
        return "No image available", 404

    processed_frame = process_image(last_frame)
    # 이미지를 JPEG 형식으로 인코딩
    ret, buffer = cv2.imencode('.jpg', processed_frame)
    if not ret:
        return "Failed to encode image", 500
    # 바이트 스트림을 메모리 파일로 변환
    image_stream = io.BytesIO(buffer.tobytes())
    #----------------------------------------------
    # 이미지 파일을 클라이언트에 반환
    return send_file(image_stream, mimetype='image/jpeg', as_attachment=True, download_name='captured_image.jpg')
#---------------신규추가--------------------
route_came_from=[]

def process_image(image):
    global route_came_from
    binary_image = preprocess_image(image)
    start_end_points = get_start_end_points(binary_image)
    #start, end = start_end_points

    walls = set()
    h, w = binary_image.shape
    for y in range(h):
        for x in range(w):
            if binary_image[y, x] == 0:  # Assuming black pixels are walls
                walls.add((y, x))

    color_image = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)

    process_image_astar,route_came_from=convert_a_star(color_image, start_end_points, walls)
    print(route_came_from)

    return process_image_astar

#----------------좌표-----ASTAR------------------
@bp.route('/route_data', methods=['POST'])
def route_data():
    global route_came_from
    if route_came_from is None:
        return jsonify({"error": "No route data available"}), 404
    
    return jsonify(route_came_from)

@bp.route('/arrow', methods=['POST'])
def arrow():
    global trans_end_red_position , green_to_red_angle,newh,neww,not_trance_end_red_position,not_trance_center_green_position
    if route_came_from is None:
        return jsonify({"error": "No route data available"}), 404
    #angle은 화살표의 각도이고 이 각도는 중심점과 가장먼점 사이의 벤터의 방향 임
    return jsonify({
        "trans_end_red_position":trans_end_red_position,
        "green_to_red_angle" : green_to_red_angle,
        "newh" : newh,
        "neww" : neww,
        "not_trance_end_red_position" : not_trance_end_red_position,
        "not_trance_center_green_position":not_trance_center_green_position
        
    })


def write_to_shared_memory():
    global trans_end_red_position, green_to_red_angle, newh, neww, not_trance_end_red_position, not_trance_center_green_position

    size = 1024  # 메모리 크기
    status_size = 1  # 상태를 저장할 1바이트
    filepath = "shared_memory.dat"

    # 파일이 존재하지 않으면 생성
    if not os.path.exists(filepath):
        with open(filepath, "wb") as f:
            f.write(b'\x00' * (size + status_size))

    with open(filepath, "r+b") as f:
        mm = mmap.mmap(f.fileno(), size + status_size)

        try:
            # 상태를 '쓰기 중'으로 설정
            if mm[0] == 1:  # 이전 데이터가 아직 읽히지 않았다면 함수를 종료
                print("메모리 쓰기 작업이 진행 중입니다. 현재 쓰기를 건너뜁니다.")
                return
            data_dict = {}
            data_dict = {
                "trans_end_red_position": trans_end_red_position,
                "green_to_red_angle": green_to_red_angle,
                "newh": newh,
                "neww": neww,
                "not_trance_end_red_position": not_trance_end_red_position,
                "not_trance_center_green_position": not_trance_center_green_position
            }
            serialized_data = pickle.dumps(data_dict)

            # 직렬화된 데이터가 메모리 크기를 초과하는지 확인
            if len(serialized_data) > size:
                print("직렬화된 데이터가 너무 큽니다.")
                return

            mm[0] = 1  # 상태를 '쓰기 중'으로 설정
            mm.seek(status_size)
            mm.write(serialized_data.ljust(size, b'\x00'))  # 데이터 작성
            mm[0] = 0  # 상태를 '쓰기 완료'로 설정

            print("Data written to shared memory:", data_dict)
        except Exception as e:
            print(f"에러 발생: {e}")
        finally:
            mm.close()

def background_write_to_shared_memory():
    global g_captrue_memory_start
    while True:
        if g_captrue_memory_start == 1:
            write_to_shared_memory()
            time.sleep(0.001)

# 백그라운드 스레드 시작
threading.Thread(target=background_write_to_shared_memory, daemon=True).start()
