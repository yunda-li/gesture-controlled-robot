import cv2
import numpy as np

template = cv2.imread('/home/yundali/Duckiebot/ME740_final/packages/testing/src/duck_template.png')


#Values in BGR
lower_bound = np.array([0, 0, 100]) # B and G are less than your max, R is above your min
upper_bound = np.array([100, 160, 255]) # B max, G max, R can go up to 255


mask = cv2.inRange(template, lower_bound, upper_bound)




cv2.imwrite('/home/yundali/Duckiebot/ME740_final/packages/testing/src/duck_mask.png', mask)