import numpy as np
import random
import math
import cv2

# 이미지 변환(small_test1_result.png)
#img_path = '/Users/jungwoojoo/Desktop/Study/kimiro/Soft_Robotics_Lab/RRT_algo/small_test1_result.png'
# 이미지 변환(test1_result.png)
img_path = '/Users/jungwoojoo/Desktop/Study/kimiro/Soft_Robotics_Lab/RRT_algo/test1_result.png'
#이미지 읽어오기
img = cv2.imread(img_path,cv2.IMREAD_GRAYSCALE) #그레이스케일로 읽어오기
#이미지 이진화 (흑백으로 변환)
#128기준으로 흰색(255),검은색(0) 설정
_, binary_image = cv2.threshold(img,128,255,cv2.THRESH_BINARY)

# 원 구역 안의 시작노드와 목적지노드 위치 설정(small_test1_result.png용)
#start_node = (80,80)
#dest_node = (640,370)
#원 구역 안의 시작 노드와 목적지 노드 위치 설정(test1_result.png)
start_node = (400,1000)
dest_node = (2500,300)


#노드클래스
class Node:
    #생성자
    def __init__(self,position,parent=None):
        self.position = position
        self.parent = parent

# 노드가 유효공간에 위치하는지 확인하는 함수
def is_valid(point, binary_image):
    x, y = point
    # p(점)가 이미지 픽셀 안에 위치하고, 흰색영역에 있는지 확인해 모두 만족하는 경우에만 true 반환
    return 0 <= x < binary_image.shape[1] and 0 <= y < binary_image.shape[0] and binary_image[y, x] == 255 

# 가장 가까운 노드를 찾아주는 함수
def nearest_node(nodes, point):
    nearest = nodes[0] #가장가까운 노드 변수
    min_d = float('inf')  # min_d의 초기값을 무한대로 설정
    for n in nodes: #nodes를 돌면서 point와 가장 가까운 노드를 찾기
        d = np.linalg.norm(np.array(n.position) - np.array(point))
        if d < min_d:
            nearest = n
            min_d = d
    return nearest

# 노드에서 지점으로 스텝 이동 (p1에서 p2방향으로 max_step만큼 이동한 지점을 반환)
def step(p1, p2, max_step=30): #원래는 max_step = 10이였음
    #두 점 사이 거리가 max_step보다 작을 경우에는 p2를 반환
    if np.linalg.norm(np.array(p2) - np.array(p1)) < max_step:
        return p2
    else:
        t = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        return (int(p1[0] + max_step * math.cos(t)), int(p1[1] + max_step * math.sin(t)))

# RRT 함수
def rrt(binary_image, start_node, dest_node, max_iter=10000):
    nodes = [Node(start_node)] #start_node의 정보로 Node 객체 생성
    for i in range(max_iter): #max_iter동안 반복
        rand_point = (random.randint(0, binary_image.shape[1] - 1), random.randint(0, binary_image.shape[0] - 1)) #이미지 내의 무작위 점을 선정
        nearest_n = nearest_node(nodes, rand_point) #가장 가까운 노드 찾기
        new_point = step(nearest_n.position, rand_point) #새로운 포인트 만들기(max_step만큼 이동)

        if is_valid(new_point, binary_image): #새 포인트가 유효한지확인
            new_node = Node(new_point, nearest_n)
            nodes.append(new_node)

            if np.linalg.norm(np.array(new_point) - np.array(dest_node)) < 10: #새로 생성된 노드가 목적지 노드와 가까우면,
                return new_node, nodes #new_node와 nodes 리스트를 반환
    return None, nodes #목적지에 도달하지 못했을 경우 (None 반환)

# 경로 추적 함수
def get_path(last_node):
    path = [] #경로를 담을 리스트
    current_node = last_node #현재 노드는 마지막 노드
    while current_node is not None: #시작노드에 도달할때까지
        path.append(current_node.position) #현재 노드 위치를 path에 추가
        current_node = current_node.parent #current_node를 부모노드로
    return path[::-1] #경로를 역순으로 반환

start_point = start_node
dest_point = dest_node

last_node , nodes = rrt(binary_image, start_point, dest_point) #rrt 호출

if last_node is not None:
    path = get_path(last_node)
    for i in range(len(path)-1):
        cv2.line(binary_image,path[i],path[i+1],(0,255,0),2)  #두 점을 이어서 경로 선을 만들기

output_image = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR) #이진화된 흑백 이미지를 BGR 컬러 이미지로 변환
for node in nodes:
    cv2.circle(output_image, node.position,2,(255,0,0),-1) #각노드의 위치에 원을 그리기

cv2.circle(output_image,start_point,5,(0,0,255),-1) #시작 노드 점 찍기
cv2.circle(output_image,dest_point,5,(0,255,0),-1) #목적지 노드 점 찍기

cv2.imshow('RRT path',output_image) #시각화 창 제목
cv2.waitKey(0) #아무 키를 누르는걸 기다림
cv2.destroyAllWindows() #창 닫기




