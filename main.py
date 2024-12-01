# File to convert depth image to navigation
import cv2
import image_filtering
import algorithm
import os
from pathlib import Path

def main():
    
    #Hey! Plz Take Midas Pics
        # img = cv2.imread("output\\IMG_20230424_124619-dpt_swin2_tiny_256.png")
        # img = cv2.GaussianBlur(img, (5, 5), 0)
        # cv2.imshow("img", img)
        # cv2.waitKey(0)

    input_folder = Path('input')
    # output_folder = Path('output')
    supported_types = ['.jpeg', '.jpg', '.png']
    image_files = [os.path.join(input_folder, f) for f in os.listdir(input_folder) if os.path.splitext(f)[1] in supported_types]

    for image_file in image_files:
        print(image_file)
        image_file = cv2.imread(image_file)
        # image_file = cv2.resize(image_file, (600, 600), cv2.interpolation=cv2.INTER_AREA)
        # cv2.imshow("img", image_file)
        # cv2.waitKey(0)
    
        # image_file = cv2.GaussianBlur(image_file, (3, 3), 0)
        img = image_filtering.apply_laplacian(image_file)

        img, h, w, _ = image_filtering.apply_thresholding(img)

        img= image_filtering.apply_dilation(img, h, w)

        img2 = img.copy()
        center, sightly_left, sightly_right, left, right, img2 = algorithm.apply_algorithm(img, img2, h, w)
        # center,  left, right, img2 = algorithm5.apply_algorithm(img, img2, h, w)

        decision = algorithm.apply_left_right(center, sightly_left, sightly_right, left, right)
        print("Decision: ", decision)
        print("Decision: ", decision)
        print("Decision: ", decision)
        print("Decision: ", decision)

        # output_file = os.path.join(output_folder, os.path.basename(image_file))
        # cv2.imwrite(output_file, img)

# #########################################################################################
# #Testing
        
        # img = cv2.resize(img, (600, 600))
        cv2.line(img, (0, int(h * 0.645)), (w, int(h * 0.645)), (0, 255, 255), 2) 
        cv2.line(img, (0, int(h * 0.645) + 25), (w, int(h * 0.645) + 25), (0, 255, 255), 2) 
        cv2.imshow('output', img2) 
        
        # cv2.resizeWindow('output', 600, 600)  
        cv2.waitKey(0)
if __name__ == "__main__":
    main()
