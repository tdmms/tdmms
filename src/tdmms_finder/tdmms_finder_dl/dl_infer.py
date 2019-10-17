import requests
import json
import csv
import pprint
import sys
import os

args = sys.argv
filename = args[1]
filename_roi = '/ramdisk/tmp_roi.csv'
filename_json = '/ramdisk/tmp.json'
filename_scores = '/ramdisk/tmp_scores.csv'
filename_masks = '/ramdisk/tmp_masks.csv'
filename_class = '/ramdisk/tmp_class.csv'
url_dlserver = os.getenv("DLSERVER_URL", "http://localhost:5000")

data = open(filename, 'rb').read()
r = requests.post(url=url_dlserver, data=data)
ret = r.json()

#print(ret['masks'])
# Mask_RCNN Sometimes returns empty mask with 
# finite classes and rois 
# the following code is required to avoid the 
# errors regarding this issue 

try:
    with open(filename_json, mode='w') as f:
        f.write(json.dumps(ret))

    classes = json.loads(ret['class'])
    with open(filename_class, mode='w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(classes)

    rois = ret['rois']
    with open(filename_roi, mode='w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerows(json.loads(rois))

    scores = ret['scores']
    with open(filename_scores, mode='w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(json.loads(scores))

    masks = json.loads(ret['masks'])
    masks_csv = []
    for i, (classnum, mask) in enumerate(zip(classes, masks)):
        for j, mask_elem in enumerate(mask[str(classnum)]):
            mask_elem_dic = mask_elem['geometry']
            for mask_elem_dic_g in mask_elem_dic:
                masks_csv.append([i,j,classnum,mask_elem_dic_g['x'],mask_elem_dic_g['y']])

    with open(filename_masks, mode='w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerows(masks_csv)

except KeyError:
    print('Parse error!!')
    with open(filename_class, mode='w') as f:
        f.write('')
    with open(filename_roi, mode = 'w') as f:
        f.write('')
    with open(filename_scores, mode ='w') as f:
        f.write('')
    with open(filename_masks, mode='w') as f:
        f.write('')
    sys.exit()
