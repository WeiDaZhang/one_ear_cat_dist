import os
import traceback
from datetime import datetime
import cv2 as cv
import numpy as np

def load_img(file_namepath, dspl_wnd_name = ""):
    image = cv.imread(	file_namepath )
    if dspl_wnd_name:
        cv.imshow(dspl_wnd_name, image)
        cv.waitKey(0)
    return image

def img_rm_channel(img):
    if len(img.shape) == 2:
        return img
    n_chn = img.shape[2]
    if n_chn == 1:
        img = np.squeeze(img)
        return img
    img = np.squeeze(img[:,:,0])
    #print("Shape: {}".format(img.shape))
    return img

def unify_img_size(img_list):
    size_list = list()
    same_size_img_list = list()
    for img in img_list:
        size_list.append(img.shape)
    size_list = np.array(size_list)
    min_size = np.min(size_list, 0)
    for img in img_list:
        diff_size = img.shape - min_size
        while diff_size[0]:
            img = np.delete(img, img.shape[0] - 1, 0) 
            diff_size[0] -= 1
        while diff_size[1]:
            img = np.delete(img, img.shape[1] - 1, 1) 
            diff_size[1] -= 1
        same_size_img_list.append(img)
    return same_size_img_list

def create_folder(parent_path, folder_name):
    create_folder_path = os.path.join(parent_path, folder_name)
    # Create the directory 'image_data'
    try:
        os.makedirs(create_folder_path, exist_ok=True)
        print("Directory {} created successfully".format(create_folder_path))
        return create_folder_path
    except OSError as error:
        print("Directory {} can not be created".format(create_folder_path))
        return ""


def record_img_func_names(img_stack, file_prefix = ''):
    cd_dir_str = os.getcwd()
    image_data_str = "image_data"
    
    image_data_path = create_folder(cd_dir_str, image_data_str)
    if not image_data_path:
        return False

    date_str = datetime.today().strftime("%Y_%m_%d")
    image_data_date_path = create_folder(image_data_path, date_str)
    if not image_data_date_path:
        return False

    #same_date_folder_list = [name for name in os.listdir(image_data_date_path) if os.path.isdir(os.path.join(image_data_date_path, name))]
    #largest_folder = 0
    #for folder_name in same_date_folder_list:
    #    try:
    #        value = int(folder_name)
    #    except ValueError as ex:
    #        continue

    stack = traceback.extract_stack()
    procname_list = []
    for layer in stack:
        procname_list.append(layer[2])

    procname_in_run = []
    for procname in procname_list[-2::-1]:
        if procname == "<module>":
            break
        else:
            procname_in_run.append(procname)

    if not procname_in_run:
        procname_in_run.append("local")

    proc_folder_str = "_".join(procname_in_run)

    image_data_date_proc_path = create_folder(image_data_date_path, proc_folder_str)
    if not image_data_date_proc_path:
        return False

    for idx, img in enumerate(img_stack):
        if file_prefix:
            file_prefix = file_prefix + "_"
        cv.imwrite(image_data_date_proc_path + '\\' + file_prefix + str(idx) + '.bmp', img)

if __name__ == '__main__':

    record_img_func_names([np.zeros((5,5))])