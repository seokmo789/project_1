import sys
import numpy as np
from config import config
from collections import deque
import time
import cv2  # OpenCV 라이브러리 추가
from queue import PriorityQueue
import copy

def manhattan_distance(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def is_valid(mat, row, col):
    M, N = mat.shape
    return (row >= 0) and (row < M) and (col >= 0) and (col < N) \
        and mat[row][col] == 0

def reconstruct_path(came_from, start, end):
    path = [end]
    while end in came_from:
        end = came_from[end]
        path.append(end)
    path.reverse()
    return path


def a_star(win, startEnd, walls, size = None):
    start, end = startEnd
    hSize = win.shape[0] // config['board']['h']
    wSize = win.shape[1] // config['board']['w']
    #hSize = 16
    #wSize = 16
    mat = np.zeros([config['board']['h'], config['board']['w']])  # 수정된 보드 크기
    mat = np.copy(walls)  # walls를 mat로 복사
    #print(walls)
    #print(mat)
    # explore 4 neighbors
    row = [-1, 0, 0, 1]
    col = [0, -1, 1, 0]

    q = PriorityQueue()
    count = 0
    q.put((0, count, start))
    g_score = {}
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            g_score[(i, j)] = float('inf')

    f_score = copy.deepcopy(g_score)
    g_score[start] = 0
    f_score[start] = manhattan_distance(*start, *end)
    q_hash = {start}
    came_from = {}
    while not q.empty():
        current = q.get()[2]
        q_hash.remove(current)

        for k in range(4):
            coordinate = (current[0] + row[k], current[1] + col[k])

            if is_valid(mat, *coordinate):
                temp_g_score = g_score[current] + 1
                if temp_g_score < g_score[coordinate]:
                    came_from[coordinate] = current
                    g_score[coordinate] = temp_g_score
                    f_score[coordinate] = temp_g_score + manhattan_distance(*coordinate, *end)
                    if coordinate not in q_hash:
                        count += 1
                        q_hash.add(coordinate)
                        q.put((f_score[coordinate], count, coordinate))
                        if 0 <= coordinate[0] < win.shape[0] and 0 <= coordinate[1] < win.shape[1]:
                            #cv2.rectangle(win, (coordinate[0]*hSize, coordinate[1]*wSize), ((coordinate[0]+1)*hSize, (coordinate[1]+1)*wSize), (0, 255, 0), -1)
                            cv2.rectangle(win, (coordinate[1]*wSize, coordinate[0]*hSize), ((coordinate[1]+1)*wSize, (coordinate[0]+1)*hSize), (0, 255, 0), -1)
                        cv2.imshow('A* Pathfinding', win)
                        cv2.waitKey(1)

        
        
        if current == end:
            path = reconstruct_path(came_from, start, end)
            count = 0
            #cv2.circle(win, (end[1]*wSize + wSize//2, end[0]*hSize + hSize//2), min(wSize, hSize)//2, (0, 0, 255), -1)  # 도착 노드 표시
            
            # 경로 시각화
            for node in path:
                cv2.rectangle(win, (node[1]*wSize, node[0]*hSize), ((node[1]+1)*wSize, (node[0]+1)*hSize), (255, 0, 0), -1)
                count += 1
                cv2.imshow('A* Pathfinding', win)
                cv2.waitKey(5)
            cv2.circle(win, (end[1]*wSize + wSize//2, end[0]*hSize + hSize//2), min(wSize, hSize)//2, (0, 0, 255), -1)  # 도착 노드 표시
            print(came_from)
            cv2.putText(win, f"The shortest path from source to destination has length {count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            return win, path

    if q.empty() and current != end:
        cv2.putText(win, "Destination can't be reached from a given source", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return win, path
    '''
    
        
        
        
        if current == end:
            count = 0
            cv2.circle(win, (end[1]*wSize + wSize//2, end[0]*hSize + hSize//2), min(wSize, hSize)//2, (0, 0, 255), -1)  # 빨간색으로 표시
            #cv2.circle(win, (end[0]*hSize + hSize//2, end[1]*wSize + wSize//2), min(wSize, hSize)//2, (0, 0, 255), -1)  # 빨간색으로 표시
            path = reconstruct_path(came_from, start, end)
            while current in came_from:
                current = came_from[current]
                cv2.rectangle(win, (current[1]*wSize, current[0]*hSize), ((current[1]+1)*wSize, (current[0]+1)*hSize), (255, 0, 0), -1)
                count += 1
                cv2.imshow('A* Pathfinding', win)
                cv2.waitKey(5)
            print(came_from)
           
            return win,path
            #break
    if q.empty() and current != end:

        return win,path
    '''


def convert_a_star(win, startEnd, walls):
    Size = (win.shape[0], win.shape[1])
    #print(startEnd)
    newh = (Size[0] // config['board']['h'])
    neww = (Size[1] // config['board']['w'])
    newstartEnd = ((startEnd[0][1]//newh, startEnd[0][0]//neww), (startEnd[1][1]//newh, startEnd[1][0]//neww))
    print(newstartEnd)

    for i in range(config['board']['h']):
        cv2.line(win, (0, i*newh), (Size[1], i*newh), (125, 125, 125), 1)
    for j in range(config['board']['w']):
        cv2.line(win, (j*neww, 0), (j*neww, Size[0]), (125, 125, 125), 1)
    
    resized_walls = np.zeros((config['board']['h'], config['board']['w']), dtype=int)
    for wall in walls:
        new_wall = (wall[0] // newh, wall[1] // neww)
        resized_walls[new_wall] = 1
   
    #print(resized_walls)
    for i in range(config['board']['h']):
        for j in range(config['board']['w']):
            if resized_walls[i][j] == 1:
                cv2.rectangle(win, (j*neww, i*newh), ((j+1)*neww, (i+1)*newh), (255,215,0), -1)
    cv2.circle(win, (newstartEnd[0][1]*neww + neww//2, newstartEnd[0][0]*newh + newh//2), min(neww, newh)//2, (0,100,0), -1)
    cv2.circle(win, (newstartEnd[1][1]*neww + neww//2, newstartEnd[1][0]*newh + newh//2), min(neww, newh)//2, (138,43,226), -1)    
    
    process_image_fin,route_came_from=a_star(win, newstartEnd, resized_walls)
    return process_image_fin,route_came_from

#--------------------------------------------------------------------------공의 위치 좌표 변환 ----------------------------
'''
def ball_convert_a_star(win, ball_position):
    Size = (win.shape[0], win.shape[1])
    newh = (Size[0] // config['board']['h'])
    neww = (Size[1] // config['board']['w'])
    ball_newstart = (ball_position[0] // neww,ball_position[1] // newh)
    print(ball_newstart)

    return ball_newstart
'''

def ball_convert_a_star(win, ball_position):
 
    if not isinstance(ball_position, tuple) or len(ball_position) != 2:
        raise ValueError("ball_position must be a tuple of length 2 (x, y)")

    Size = (win.shape[0], win.shape[1])
    newh = Size[0] // config['board']['h']
    newh_flot=Size[0] / config['board']['h']
    newh_flot=round(newh_flot,3)

    neww = Size[1] // config['board']['w']
    neww_flot=Size[1] / config['board']['w']
    neww_flot=round(neww_flot,3)


    ball_newstart = (int(ball_position[0] // neww), int(ball_position[1] // newh))
    # Return the coordinates as a tuple
    return ball_newstart,neww_flot,newh_flot