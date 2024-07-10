import glob
import re
import os
import cv2

def crop_rectangle(path):
    target = os.path.join(os.path.dirname(path), 'clopped')
    target_file = os.path.basename(path)
    #target_file = target_file.replace('.bvh', '_edited.bvh')
    if not os.path.exists(target):
        os.mkdir(target)
    x = 725  # 長方形の左上のx座標
    y = 100   # 長方形の左上のy座標
    width = 1175  # 長方形の幅
    height = 900  # 長方形の高さ  
    # 画像の読み込み
    image = cv2.imread(path)
    # 指定した長方形の領域を切り抜く
    cropped_image = image[y:y+height, x:x+width]    
    # 切り抜いた画像を保存
    cv2.imwrite(os.path.join(target,target_file), cropped_image)
     
source_path=r'C:\Users\sayak\Laboratory\Picture'
for src_file in glob.glob(os.path.join(source_path,'*.png'), recursive=False):
    crop_rectangle(src_file)