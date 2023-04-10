from lu_vp_detect import VPDetection
import time
import cv2

length_thresh = 80
principal_point = None
focal_length = 959.791
seed = 1337

img = './image.png'

vpd = VPDetection(length_thresh, principal_point, focal_length, seed)

time_start = time.time()
vps = vpd.find_vps(img)
time_end = time.time()
print('time cost', time_end - time_start, 's')
# print(vps)

img_cV2 = cv2.imread(img)
img_shape = cv2.imread(img).shape

vps = vpd.vps

vps_2D = vpd.vps_2D

# print(vps_2D)
# print(vps)
# vpd.create_debug_VP_image(show_image=True, save_image='debug.png')

for vp in vps_2D:
    # check this point is in the image
    if vp[0] < 0 or vp[0] > img_shape[1] or vp[1] < 0 or vp[1] > img_shape[0]:
        continue
    cv2.circle(img_cV2, (int(vp[0]), int(vp[1])), 5, (0, 0, 255), -1)
    cv2.putText(img_cV2, str(vp), (int(vp[0]), int(vp[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    print(vp)
    
cv2.imshow('img', img_cV2)
cv2.waitKey(0)
