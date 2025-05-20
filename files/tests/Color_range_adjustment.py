import cv2
import numpy as np

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel_bgr = img[y, x]
        pixel_hsv = cv2.cvtColor(np.uint8([[pixel_bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        print(f"Position: ({x}, {y})")
        print(f"BGR: {pixel_bgr}")
        print(f"HSV: {pixel_hsv}")

cap = cv2.VideoCapture(1)
ret, img = cap.read()

cv2.imshow("Image", img)
cv2.setMouseCallback("Image", mouse_callback)

cv2.waitKey(0)
cv2.destroyAllWindows()
    