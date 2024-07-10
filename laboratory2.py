import glob
import re
import os
import cv2

folder_name = "Waisttortion"
folder_path = r'C:\Users\sayak\Laboratory\Picture'
     
source_path = os.path.join(folder_path, folder_name)

files = []

for src_file in glob.glob(os.path.join(source_path,'*.png'), recursive=False):
    image = cv2.imread(src_file)
    files.append(image)

h_image1 = cv2.hconcat(files[:5])
h_image2 = cv2.hconcat(files[5:])

h_image3 = cv2.hconcat(files)

array = [h_image1, h_image2]

image = cv2.vconcat(array)

path_10_1 = os.path.join(folder_path, '10_1', folder_name + '.png')
path_5_2 = os.path.join(folder_path, '5_2', folder_name + '.png')

cv2.imwrite(path_10_1, h_image3)
cv2.imwrite(path_5_2, image)

