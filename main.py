import numpy as np
import random
import math
import cv2

# 이미지 변환
img_path = '/Users/jungwoojoo/Desktop/Study/kimiro/Soft_Robotics_Lab/RRT_algo/small_test1_result.png'

#이미지 읽어오기
img = cv2.imread(img_path,cv2.IMREAD_GRAYSCALE)
#이미지 이진화
_, binary_image = cv2.threshold(img,128,255,cv2.THRESH_BINARY)

# 원 구역 안의 시작노드와 목적지노드 위치 설정
start_node = (80,80)
dest_node = (420,320)

class Node:
    #생성자
    def __init__(self,position,parent=None):
        self.position = position
        self.parent = parent
    #노드가 유효공간에 위치하는지 확인하는 함수
    def is_valid(p,binary_image):
        x,y = p
        #p(점)가 이미지 픽셀 안에 위치하고, 흰색영역에 있는지 확인해 모두 만족하는 경우에만 true 반환
        return 0 <= x < binary_image.shape[1] and 0 <= y < binary_image.shape[0] and binary_image[y,x] == 255 
    #가장 가까운 노드를 찾아주는 함수
    def nearest_node(nodes,p):
        nearest_node = nodes[0]
        min_d = float('inf') #min_d의 초기값을 무한대로 설정
        for n in nodes:
            d = np.linalg.norm(np.array(n.position)-np.array(p))
            if d < min_d:
                nearest_node = n
                min_d = d
        return nearest_node
