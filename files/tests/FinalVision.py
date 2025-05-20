import cv2
from time import *
import numpy as np


class Vision():
    def __init__(self):
        
        self.cap = None

        # Define battery molds (rectangles)
        rectwidth = 35
        recthigh = 230
        self.cell_mold = [
            (710, 535, rectwidth, recthigh), (650, 535, rectwidth, recthigh),
            (590, 535, rectwidth, recthigh), (530, 535, rectwidth, recthigh),
            (465, 535, rectwidth, recthigh), (400, 535, rectwidth, recthigh),
            (340, 535, rectwidth, recthigh), (270, 535, rectwidth, recthigh),
            (210, 535, rectwidth, recthigh), (145, 535, rectwidth, recthigh),
            (80, 535, rectwidth, recthigh),  (25, 535, rectwidth, recthigh)
        ]

        self.box_mold = [
            (600, 25, 40, 200), (440, 25, 45, 200),
            (290, 30, 45, 200)
        ]

        self.lid_mold = [
            (580, 270, 40, 200) , (430, 270, 45, 200),
            (280, 270, 45, 200)
        ]

        self.cell_detected = 0
        self.cell_mold_detected = []
        self.box_mold_detected = []
        self.lid_mold_detected = []
        self.cell_mold_empty = []
        self.box_mold_empty = []
        self.lid_mold_empty = []

        self.cap = cv2.VideoCapture(1)


    def start_up(self):
        ret, frame = self.cap.read()

        if not ret:
            print("Error in the image.")
            self.cap.release()
            exit()

        # Perspective correction
        initialpoint = np.float32([[90, 30], [530, 190], [15, 320], [590, 420]])
        finalpoint = np.float32([[0, 0], [800, 0], [0, 800], [800, 800]])
        M = cv2.getPerspectiveTransform(initialpoint, finalpoint)
        self.dst = cv2.warpPerspective(frame, M, (800, 800)) 

    # Show image
    def view (self):
        
        cv2.imshow('Estantería con detección', self.dst)
        cv2.waitKey(0)


    # Function to detect red colour in a region
    def detect_cell_colour(self, cell_rect):
        hsv = cv2.cvtColor(cell_rect, cv2.COLOR_BGR2HSV)

        # Red ranges (two zones in HSV)
        low_red1 = np.array([0, 100, 100])
        high_red1 = np.array([10, 255, 255])
        low_red2 = np.array([160, 100, 100])
        high_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, low_red1, high_red1)
        mask2 = cv2.inRange(hsv, low_red2, high_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        return cv2.countNonZero(red_mask)
    
     # Function to detect white colour in a region
    def detect_white_colour(self, rect):
        hsv = cv2.cvtColor(rect, cv2.COLOR_BGR2HSV)

        low_white = np.array([0, 0, 200])
        high_white = np.array([180, 30, 255])

        white_mask = cv2.inRange(hsv, low_white, high_white)

        return cv2.countNonZero(white_mask)
    
    # Function to detect grey colour in a region
    def detect_grey_colour(self, rect):
        hsv = cv2.cvtColor(rect, cv2.COLOR_BGR2HSV)

        low_grey = np.array([100, 20, 120])
        high_grey = np.array([120, 110, 170])

        grey_mask = cv2.inRange(hsv, low_grey, high_grey)

        return cv2.countNonZero(grey_mask)
    
    # Function to detect orange colour in a region
    def detect_orange_colour(self, rect):
        hsv = cv2.cvtColor(rect, cv2.COLOR_BGR2HSV)

        low_orange = np.array([10, 50, 200])
        high_orange = np.array([25, 255, 255])

        orange_mask = cv2.inRange(hsv, low_orange, high_orange)

        return cv2.countNonZero(orange_mask)
    
    def detect_cell(self):
        self.cell_mold_detected = []
        for i, (x, y, w, h) in enumerate(self.cell_mold):
            rect = self.dst[y:y+h, x:x+w]
            red_area = self.detect_cell_colour(rect)
            white_area = self.detect_white_colour(rect)

            # Detect battery only if both red and white areas are present
            if red_area > 50 and white_area > 50:
                self.cell_detected += 1
                self.cell_mold_detected.append(i + 1)  # Save mold number
                cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(self.dst, 'Cell', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            else:
                self.cell_mold_empty.append(i + 1)  # Save empty molds
                cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    def detect_box(self):
        self.box_mold_detected = []
        for i, (x, y, w, h) in enumerate(self.box_mold):
            rect = self.dst[y:y+h, x:x+w]
            
            if i == 0:  # Primer rectángulo: debe detectar naranja
                orange_area = self.detect_orange_colour(rect)
                if orange_area > 6000:
                    self.box_mold_detected.append(i + 1)  # Save mold number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Naranja
                    cv2.putText(self.dst, 'Orange Box', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.box_mold_empty.append(i + 1)  # Save empty molds
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            elif i == 1:  # Segundo rectángulo: debe detectar gris
                grey_area = self.detect_grey_colour(rect)
                if grey_area > 500:
                    self.box_mold_detected.append(i + 1)  # Save mold number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Gris
                    cv2.putText(self.dst, 'Grey Box', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.box_mold_empty.append(i + 1)  # Save empty molds
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            elif i == 2:  # Tercer rectángulo: debe detectar blanco
                white_area = self.detect_white_colour(rect)
                if white_area > 6000:
                    self.box_mold_detected.append(i + 1)  # Save mold number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Blanco
                    cv2.putText(self.dst, 'White Box', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.box_mold_empty.append(i + 1)  # Save empty molds
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            else:
                self.box_mold_empty.append(i + 1)  # Save mold number
                cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


    def detect_lid(self):
        self.lid_mold_detected = []
        for i, (x, y, w, h) in enumerate(self.lid_mold):
            rect = self.dst[y:y+h, x:x+w]

            # Detect color based on the specific rectangle
            if i == 0:  # Primer rectángulo: debe detectar naranja
                orange_area = self.detect_orange_colour(rect)
                if orange_area > 6000:
                    self.lid_mold_detected.append(i + 1)  # Save lid number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Naranja
                    cv2.putText(self.dst, 'Orange Lid', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.lid_mold_empty.append(i + 1)  # Save empty lids
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            elif i == 1:  # Segundo rectángulo: debe detectar gris
                grey_area = self.detect_grey_colour(rect)
                if grey_area > 500:
                    self.lid_mold_detected.append(i + 1)  # Save lid number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Gris
                    cv2.putText(self.dst, 'Grey Lid', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.lid_mold_empty.append(i + 1)  # Save empty lids
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            elif i == 2:  # Tercer rectángulo: debe detectar blanco
                white_area = self.detect_white_colour(rect)
                if white_area > 6000:
                    self.lid_mold_detected.append(i + 1)  # Save lid number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Blanco
                    cv2.putText(self.dst, 'White Lid', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.lid_mold_empty.append(i + 1)  # Save empty lids
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            else:
                self.lid_mold_empty.append(i + 1)  # Save empty lids
                cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

if __name__ == "__main__":
   
    vision = Vision()

    while True:

        vision.start_up()
        vision.detect_cell()
        vision.detect_box()
        vision.detect_lid()

        print("Detected cells:", vision.cell_mold_detected)
        print("Detected boxes:", vision.box_mold_detected)
        print("Detected lids:", vision.lid_mold_detected)

        vision.view()
        
        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()