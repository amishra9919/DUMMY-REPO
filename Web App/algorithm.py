import numpy as np
import cv2
import random

def apply_algorithm(img, img2, h, w):
    arr = np.array([])                    # for appending 0 and 1 according to b,g,r
    x_increment = 0                       #it is x co-ordinate where we are extracting b, g, r in horizontal

    while x_increment <= w:
        if x_increment == 600:
            x_increment = 598

        # print("X is at: ", x_increment)

        count = 0 # for counting the 5 b, g, r occured
        
        # cv2.line(img, (0, int(h * 0.645)), (w, int(h * 0.645)), (0, 255, 255), 2) 
        # cv2.line(img, (0, int(h * 0.645) + 25), (w, int(h * 0.645) + 25), (0, 255, 255), 2) 
        # cv2.imshow('img', img)

        # obstacle_height_count = 5
        obstacle_height = int(h * 0.645)
        # for _ in range(obstacle_height_count):


        while obstacle_height <= h:
            b, g, r = img[obstacle_height, x_increment]
            # print(b, g, r)
            
            cv2.circle(img2, (x_increment, obstacle_height), 5, color = (0, 0, 255))
            
            if b == g == r == 255:
                # print("Entered in IF Statement")
                count = 0

            else:    
                # print("Couner:", count)                             
                count += 1
                if count == 4:
                    # print('Break')
                    break 
            obstacle_height += 5
         
            
        if count == 4:
            arr = np.append(arr, 0)  
        else:
            # print('Inside 1 Append')
            arr = np.append(arr, 1)


        x_increment = x_increment + 3

        cv2.line(img2, (0, int(h * 0.645)), (w, int(h * 0.645)), (0, 255, 255), 2) 
        cv2.line(img2, (0, h-5), (w, h-5), (0, 0, 0), 2)     
        var = random.random()
        filename = 'content' + str(var) + '.png'
        # cv2.imwrite(filename, img)

    center_size = int(len(arr) * 0.465)
    left_size = (len(arr) - center_size) // 2
    right_size = len(arr) - center_size - left_size

    left = arr[:left_size]
    center = arr[left_size:left_size+center_size]
    right = arr[-right_size:] if right_size > 0 else []

    # left_size = int(len(left) * 0.49)
    # sightly_left = left[left_size:]
    # left = left[:left_size]

    # right_size = int(len(right) * 0.49)
    # right = right[right_size:]
    # sightly_right = right[:right_size]

    slightly_left = left[27:].copy()
    slightly_right = right[0:27].copy()

    left = left[0:27].copy()
    right = right[27:].copy()



    print('center: ', center)
    print('left: ', left)
    print('sightly_left: ', slightly_left)
    print('right: ', right)
    print('sightly_right: ', slightly_right)
    print('arr', arr)



    ############################################################################################
    # Code for Left and right
    # Check for consecutive zeros in ----> i) straight ii) sightly left iii) sightly right iv) left v) right

    ############################################################################################
 

    return center, slightly_left, slightly_right, left, right, img2
    # return center, left, right, img2

def apply_left_right(center, sightly_left, sightly_right, left, right):
    # center
    c = 1
    output = 1

    if c:
        for i in range(len(center)-2):
            if center[i] == center[i+1] == center[i+2] == 0:
                output = 0
                break
        if output:
            c = 0
            return "straight"
        else:
            c = 1
            output = 1
            
    # sightly left
    if c:
        for i in range(len(sightly_left)-2):
            if sightly_left[i] == sightly_left[i+1] == sightly_left[i+2] == 0:
                output = 0        
                break
        if output:
            c = 0
            return "sl"  
        else:
            c = 1  
            output = 1 

    # sightly right
    if c:
        for i in range(len(sightly_right)-2):
            if sightly_right[i] == sightly_right[i+1] == sightly_right[i+2] == 0:
                output = 0        
                break
        if output:
            c = 0
            return "sr"
        else:
            c = 1  
            output = 1 

    # left
    if c:
        for i in range(len(left)-2):
            if left[i] == left[i+1] == left[i+2] == 0:
                output = 0        
                break
        if output:
            c = 0
            return "l" 
        else:
            c = 1  
            output = 1       

    # right
    if c:
        for i in range(len(right)-2):
            if right[i] == right[i+1] == right[i+2] == 0:
                output = 0
                break
        if output:
            c = 0
            return "r" 
        else:
            c = 1   
            output = 1

    # Stop 
    if c:
        return "stop"


