'''
FLASK  로부터 전달 받은 data를 처리 및 ESP32에게 제어신호를 전달 하는 부분 입니다.

1.FLASK 와 쉐어드 메모리로 되어있으며, FLASK와 공유한 데이터를 기반으로 벡터값을 계산합니다.
2.벡터값을 기준으로 현재 자동차가 목표 지점까지의 각도와 거리를 계산합니다.
3.계산된 결과값에 따른 제어 신호를 ESP32에 전달 합니다.

'''

import requests
import math
import mmap
import os
import pickle
import time
import numpy as np
import threading
import asyncio
# 서버의 URL 주소
server_url = 'http://192.168.1.93:5000/camera/route_data'
esp8266_url = 'http://192.168.1.111/control'

# 전역 변수
g_astar_current_position = None
background_tasks = [] 
repeat_count = 0
main_repeat_count=0
def INIT_variable():
    global g_astar_current_position, g_green_to_red_angle
    global g_trans_end_red_position, g_newh, g_neww
    global g_not_trance_end_red_position, g_not_trance_center_green_position
    global previous_distance
    global g_target_green_to_red_angle, g_target_trans_end_red_position
    global g_target_not_trance_end_red_position, g_target_not_trance_center_green_position
    global flag, Allowable_range
    global g_current_distance, g_current_angle_degrees
    global s_command_start_time
    global s_command_duration, repeat_count
    global target_reach_threshold, update_success, wait_time_for_update
 
    g_green_to_red_angle = None
    g_trans_end_red_position = None
    g_newh = None
    g_neww = None
    g_not_trance_end_red_position = None
    g_not_trance_center_green_position = None
    previous_distance = None
    g_target_green_to_red_angle = None
    g_target_trans_end_red_position = None
    g_target_not_trance_end_red_position = None
    g_target_not_trance_center_green_position = None
    flag = 0
    Allowable_range = 3
    g_current_distance = 0.0
    g_current_angle_degrees = 0.0
    s_command_start_time = None
    s_command_duration = 1
    target_reach_threshold = 40.0
    update_success = False
    wait_time_for_update = 0.02
    print("초기화 성공!")
# POST 요청을 보내는 함수
def car_move_send_func(data):
    headers = {'Content-Type': 'text/plain; charset=utf-8'}
    response = requests.post(esp8266_url, data=data, headers=headers)
    data=""

# A* 알고리즘으로 계산된 경로 데이터를 가져오는 함수
def astar_map():
    try:
        response = requests.post(server_url, json={})
        response.raise_for_status()
        route_data = response.json()
        return [[x, y] for y, x in route_data]
    except requests.RequestException as e:
        print(f"경로 데이터를 가져오는데 실패했습니다: {e}")
        return []

# 공유 메모리를 초기화하고 변수를 초기화하는 함수
def clear_shared_memory_and_initialize():
    size = 1024  # 공유 메모리의 크기
    status_size = 1
    filepath = "C:\\myserver\\myserver_flask\\shared_memory.dat"

    if os.path.exists(filepath):
        try:
            with open(filepath, "r+b") as f:
                mm = mmap.mmap(f.fileno(), size + status_size)
                mm[:] = b'\x00' * (size + status_size)  # 모든 바이트를 0으로 설정
                mm.close()  # 메모리 맵을 닫습니다.
            print("공유 메모리가 초기화되었습니다.")
        except Exception as e:
            print(f"공유 메모리 초기화 중 오류 발생: {e}")
    else:
        print("공유 메모리 파일이 존재하지 않습니다.")
    INIT_variable()

# 공유 메모리에서 데이터를 읽는 비동기 함수
async def read_from_shared_memory():
    global g_target_green_to_red_angle, g_target_trans_end_red_position, g_target_not_trance_end_red_position, g_target_not_trance_center_green_position
    global g_green_to_red_angle, g_trans_end_red_position, g_newh, g_neww, g_not_trance_end_red_position, g_not_trance_center_green_position
    size = 1024
    status_size = 1
    filepath = "C:\\myserver\\myserver_flask\\shared_memory.dat"

    if os.path.exists(filepath):
        try:
            with open(filepath, "r+b") as f:
                mm = mmap.mmap(f.fileno(), size + status_size)
                
                if flag==1: #메모리초기화 관련해서 정리
                    pass

                while mm[0] == 1:
                    await asyncio.sleep(0.001)  # 비동기적으로 잠시 대기

                mm.seek(status_size)
                serialized_data = mm.read(size).rstrip(b'\x00')
                data_dict = pickle.loads(serialized_data)

                g_green_to_red_angle = data_dict.get('green_to_red_angle', 'N/A')               
                g_trans_end_red_position = data_dict.get('trans_end_red_position', 'N/A')
                g_newh = data_dict.get('newh')
                g_neww = data_dict.get('neww')
                g_not_trance_end_red_position = data_dict.get('not_trance_end_red_position')
                g_not_trance_center_green_position = data_dict.get('not_trance_center_green_position')

                g_target_green_to_red_angle = g_green_to_red_angle
                g_target_trans_end_red_position = g_trans_end_red_position
                g_target_not_trance_end_red_position = g_not_trance_end_red_position
                g_target_not_trance_center_green_position = g_not_trance_center_green_position
                
                mm[0] = 0
        except Exception as e:
            print(f"공유 메모리에서 읽는 중 오류 발생: {e}")

# 위치를 변환하는 함수
def trance_position_func(transformed_route_data):
    global g_newh, g_neww
    if g_newh is None or g_neww is None:
        print("오류: 새로운 높이 또는 너비가 None입니다.")
        return [0, 0]

    return [transformed_route_data[0] * g_neww, transformed_route_data[1] * g_newh]

# 거리를 계산하는 함수
def get_distance_func():
    global g_target_not_trance_end_red_position, g_astar_current_position,g_target_not_trance_center_green_position
    target_position = None  # 목표 위치 초기화
    distance = 0.0 

    # 목표 위치를 초록 공의 위치로 설정
    target_position = g_target_not_trance_center_green_position  # 초록 공의 위치
    if target_position is None or g_astar_current_position is None:
        print("오류: 목표 위치 또는 현재 위치가 None입니다.")
        return 0.0

    distance = math.sqrt((target_position[0] - g_astar_current_position[0]) ** 2 +
                         (target_position[1] - g_astar_current_position[1]) ** 2)
    return round(distance, 1)

# 방향 회전 각도를 계산하는 함수
def direction_turn_angle_func():
    global g_target_not_trance_end_red_position, g_target_not_trance_center_green_position,g_astar_current_position

    # 지역 변수 초기화
    green_to_red_vector = np.array([0, 0])  # 빨간 공 방향 벡터 초기화
    green_to_astar_vector = np.array([0, 0])  # 현재 위치 방향 벡터 초기화
    angle_degrees = 0.0  # 각도 초기화

    if g_target_not_trance_end_red_position is None or g_target_not_trance_center_green_position is None:
        print("오류: 방향 계산을 위한 목표 위치가 None입니다.")
        return 0.0
    #초록공 에서 빨간 공 까지의 벡터 계산
    green_to_red_vector = np.array([
        g_target_not_trance_end_red_position[0] - g_target_not_trance_center_green_position[0],
        g_target_not_trance_end_red_position[1] - g_target_not_trance_center_green_position[1]
    ])
    #초록공 에서 목표지점(astar)까지의 벡터 계산
    green_to_astar_vector = np.array([
        g_astar_current_position[0] - g_target_not_trance_center_green_position[0],
        g_astar_current_position[1] - g_target_not_trance_center_green_position[1]
    ])
    
    arrow_end_vector_magnitude = np.linalg.norm(green_to_red_vector)
    astar_vector_magnitude = np.linalg.norm(green_to_astar_vector)
    
    dot_product = np.dot(green_to_red_vector, green_to_astar_vector)
    cos_angle = dot_product / (arrow_end_vector_magnitude * astar_vector_magnitude)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    
    angle_radians = np.arccos(cos_angle)
    angle_degrees = np.degrees(angle_radians)
    
    cross_product = green_to_red_vector[0] * green_to_astar_vector[1] - \
                    green_to_red_vector[1] * green_to_astar_vector[0]
    
    if cross_product < 0:
        angle_degrees = -angle_degrees
    angle_degrees=round(angle_degrees,1)
    
    return angle_degrees

# 명령어를 짧게 반복적으로 전송하는 비동기 함수
async def send_turn_command(target_angle):
    global s_command_start_time, g_current_angle_degrees, g_current_distance
    global wait_time_for_update,update_success
    print("명령어 보내는 함수 진입")
    
    while abs(target_angle) > 5:  # 각도 조정의 정밀도를 높임
        print("방향체크 함수: 무한반복 진입")
        
        if target_angle > 0:
            direction = 'right'
            print("r으로 회전")
            car_move_send_func('r\n'.encode('utf-8'))  # 오른쪽으로 회전 명령 전송
        else:
            direction = 'left'
            print("l으로 회전")
            car_move_send_func('l\n'.encode('utf-8'))  # 왼쪽으로 회전 명령 전송
            
        print(f"{direction}으로 점진적으로 회전을 시작합니다.")
        await asyncio.sleep(0.0001) # 8.16 이 코드가 있다면 자동차는 빠르게 동작
        #time.sleep(0.02)
        # 회전 후 정지 명령 전송
        print("정지 명령을 보냅니다.")
        car_move_send_func('s\n'.encode('utf-8'))  # 정지 명령 전송

        await asyncio.sleep(0.08)  # 회전 후 정지 시간
        
        update_success = False  # 초기화
        while not update_success:
            print("업데이트 대기 중...")
            await update_distance_and_angle()
            await asyncio.sleep(wait_time_for_update)  # 전역 변수 사용
        
        target_angle = g_current_angle_degrees
        print(f"방향체크 함수: 현재 각도: {g_current_angle_degrees}, 현재 거리: {g_current_distance}")

    s_command_start_time = None  # 명령 전송 시간 초기화

# 방향이 맞는지 확인하는 함수
def is_correct_direction():
    print("방향 체크 해보자..")
    angle_difference = 0.0  # 각도 차이 초기화
    angle_difference = abs(g_current_angle_degrees)

    return angle_difference <= 5  # 오차 범위를 5도로 설정

# 거리가 목표지점까지 도달했는지 확인하는 함수
def is_correct_distance():
    print("거리 체크 해보자..")
    
    correct_distance = 0.0  # 각도 차이 초기화
    correct_distance = g_current_distance
    print(f"거리체크함수 : 현재 거리 {correct_distance}")

    return correct_distance <= target_reach_threshold
#------------------------------------------------------------------------
#--------------------------이동 중에 방향을 모니터링 하는 함수
async def monitor_direction_while_moving():
    global previous_distance, flag, update_success  # 전역 변수 선언
    global g_current_distance, g_current_angle_degrees, s_command_start_time
    global wait_time_for_update
    print("실시간방향 체크 함수 진입")
    

    update_success = False  # 초기화
    while not update_success:
        print("업데이트 대기 중...")
        await update_distance_and_angle()
        await asyncio.sleep(wait_time_for_update)  # 전역 변수 사용
    
    if flag == 3:
        while not is_correct_distance():            
            while not is_correct_direction():
                print("실시간방향 체크 함수: Direction misaligned, adjusting...")
                target_angle = g_current_angle_degrees  # 현재 각도를 목표 각도로 설정
                await send_turn_command(target_angle)  # send_turn_command 호출

            print("실시간방향 체크 함수: 방향 맞았어 출발")
            car_move_send_func('g\n'.encode('utf-8'))  # 다시 전진 명령 전송
            print(f"실시간방향 체크 함수: 현재 각도: {g_current_angle_degrees}, 현재 거리: {g_current_distance}")
            
            update_success = False  # 초기화
            while not update_success:
                print("업데이트 대기 중...")
                await update_distance_and_angle()
                await asyncio.sleep(wait_time_for_update)  # 전역 변수 사용

       
# =======================================================자동차 이동 제어 함수
async def move_car_func():
    global previous_distance, flag
    global g_current_distance, g_current_angle_degrees, g_astar_current_position
    global s_command_start_time
    global repeat_count
    global target_reach_threshold
    global update_success,wait_time_for_update

    update_success = False  # 초기화
    # update_success가 True가 될 때까지 대기
    while not update_success:
        print("업데이트 대기 중...")
        await update_distance_and_angle()
        await asyncio.sleep(wait_time_for_update)  # 전역 변수 사용

    # 초기 속도 설정
    if flag == 0:  # 거리 변화가 있을 때까지 'u' 명령어 전송 루틴
        if previous_distance is None:
            previous_distance = g_current_distance
            return  # 초기화 완료, 다음 루프에서 거리 변화를 계속 감지

        # 거리 변화 감지
        distance_change = abs(g_current_distance - previous_distance)
        if distance_change > Allowable_range:
            if s_command_start_time is None:
                s_command_start_time = time.time()
            
            elapsed_time = time.time() - s_command_start_time
            if elapsed_time < s_command_duration:
                print("flage0: Sending 's' command")
                car_move_send_func('s\n'.encode('utf-8'))
            else:
                flag = 2
                s_command_start_time = None  # 타이머 초기화
                print("Flag set to 2 due to elapsed time for 's' command.")
                return
            return  # 종료

        # 거리 변화가 없을 때 'u' 명령어 전송
        if g_current_distance == previous_distance:
            print("flage0: u 호출")
            car_move_send_func('u\n'.encode('utf-8'))

            time.sleep(0.01)
        previous_distance = g_current_distance

    # 방향 회전 조정 루틴
    elif flag == 2:
        print("flag 2 호출")
        while not is_correct_direction():
            print("Direction is incorrect, adjusting...")
            await send_turn_command(g_current_angle_degrees)
            await asyncio.sleep(0.005)  # 전역 변수 사용
        print("flage2: Direction is correct, moving forward.")
        flag = 3
        await asyncio.sleep(0.01)  # 전역 변수 사용
   
    # 이동 및 방향 모니터링
    elif flag == 3:
        print("flag 3 호출")
        await monitor_direction_while_moving()  # 비동기로 호출

        if is_correct_distance():
            print("flage3: 도착!!!!!")
            repeat_count += 1
            print("Target reached. Stopping the car.")
            end_time = time.time() + 3
            while time.time() < end_time:
                car_move_send_func('s\n'.encode('utf-8'))
            flag = 5
            return 

# 거리와 각도를 업데이트하는 함수
async def update_distance_and_angle():
    global g_current_distance, g_current_angle_degrees, flag, g_astar_current_position, update_success
    print("업데이트 함수 : 호출됬음")
    if update_success == False:
        g_current_distance = get_distance_func()
        g_current_angle_degrees = direction_turn_angle_func()
        update_success = True  # 업데이트 성공

# 백그라운드에서 데이터를 지속적으로 업데이트하는 함수
async def background_data_update():
    while True:
        await read_from_shared_memory()

# 백그라운드 작업을 시작하는 함수
async def background_strat():
    global background_tasks
    task = asyncio.create_task(background_data_update())
    background_tasks.append(task)  # 작업을 리스트에 추가

# 모든 비동기 작업을 정리하는 함수
async def clear_all_tasks():
    global background_tasks
    for task in background_tasks:
        task.cancel()  # 각 작업 취소
        try:
            await task  # 작업이 종료될 때까지 대기
        except asyncio.CancelledError:
            print("Task was cancelled.")
    background_tasks = []  # 리스트 초기화

# 메인 함수
async def main():
    global g_astar_current_position,flag
    global background_tasks
    global g_current_distance,previous_distance,main_repeat_count
    await clear_all_tasks() 
    INIT_variable()
    await read_from_shared_memory()
    transformed_route_data = astar_map()
    print(f"astar_map: {transformed_route_data}")
    g_astar_current_position = trance_position_func(transformed_route_data[0])
    print(f"첫번째 요소{transformed_route_data[0]}")
    # 백그라운드 데이터 업데이트 태스크 시작
    await background_strat()
    
                
    while True:

        if flag == 5:
            print("클리어 진입!!!")
            main_repeat_count=main_repeat_count+1
            
            if main_repeat_count==2:
                print("함수 종료")
                break
            
            await clear_all_tasks()  # 모든 비동기 작업 종료
            clear_shared_memory_and_initialize()
            print(f"초기화된 값 : {g_green_to_red_angle}, {g_trans_end_red_position}, {g_not_trance_end_red_position}, {g_not_trance_center_green_position}")
            print("1초 후 재시작 합니다")
            await read_from_shared_memory()
            g_astar_current_position = trance_position_func(transformed_route_data[-1])
            print(f"마지막 요소{transformed_route_data[-1]}")
            await background_strat()
            while not update_success:
                print("업데이트 대기 중...")
                await update_distance_and_angle()
                await asyncio.sleep(wait_time_for_update)  # 전역 변수 사용
            await asyncio.sleep(1)
            flag=2
            
            print("start!!!")
        
        await move_car_func()  # move_car_func을 비동기적으로 실행
    
    print("종료!!")

if __name__ == '__main__':
    try:
        asyncio.run(main())  # 메인 비동기 함수 실행
    except requests.RequestException as e:
        print(f"An error occurred: {e}")