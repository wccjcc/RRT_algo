import numpy as np
import random
import math
import cv2

# 이미지 변환
img_path = '/Users/jungwoojoo/Desktop/Study/kimiro/Soft_Robotics_Lab/RRT_algo/small_test1_result.png'

img = cv2.imread(img_path)
cv2.imshow('Image',img)

cv2.waitKey(0)

cv2.destroyAllWindows()