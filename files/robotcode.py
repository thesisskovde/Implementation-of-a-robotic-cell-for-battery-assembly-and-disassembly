from robodk.robolink import * # RoboDK API
from robodk.robomath import *  # Math functions for transformations
from time import *
import numpy as np  # Import NumPy functions
import socket
import cv2

IP = "192.168.0.102"
PORT = 30002

base = __file__.rsplit("\\", 1)[0]  # Windows-only compatibitily!
GRIPPER_FOLDER_PATH = f"{base}\\local_files"
GRIPPER_OPEN_PATH = f"{base}\\gripper_open.script"
GRIPPER_OPEN_CELL_PATH = f"{base}\\gripper_open_cell.script"
GRIPPER_CLOSE_BOX_PATH = f"{base}\\gripper_close_box.script"
GRIPPER_CLOSE_CELL_PATH = f"{base}\\gripper_close_cell.script"
GRIPPER_CLOSE_LID_PATH = f"{base}\\gripper_close_lid.script"
ROBOTIC_STATION_PATH = f"{base}\\robotic_station.rdk"

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
            
            if i == 0:  # First rectangle must be orange
                orange_area = self.detect_orange_colour(rect)
                if orange_area > 6000:
                    self.box_mold_detected.append(i + 1)  # Save mold number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Naranja
                    cv2.putText(self.dst, 'Orange Box', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.box_mold_empty.append(i + 1)  # Save empty molds
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            elif i == 1:  # Second rectangle must be grey
                grey_area = self.detect_grey_colour(rect)
                if grey_area > 500:
                    self.box_mold_detected.append(i + 1)  # Save mold number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Gris
                    cv2.putText(self.dst, 'Grey Box', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.box_mold_empty.append(i + 1)  # Save empty molds
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            elif i == 2:  # Third rectangle must be white
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
            if i == 0:  # First rectangle must be orange
                orange_area = self.detect_orange_colour(rect)
                if orange_area > 6000:
                    self.lid_mold_detected.append(i + 1)  # Save lid number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Naranja
                    cv2.putText(self.dst, 'Orange Lid', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.lid_mold_empty.append(i + 1)  # Save empty lids
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            elif i == 1:  # Second rectangle must be grey
                grey_area = self.detect_grey_colour(rect)
                if grey_area > 500:
                    self.lid_mold_detected.append(i + 1)  # Save lid number
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Gris
                    cv2.putText(self.dst, 'Grey Lid', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                else:
                    self.lid_mold_empty.append(i + 1)  # Save empty lids
                    cv2.rectangle(self.dst, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(self.dst, 'Empty', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            elif i == 2:  # Third rectangle must be white
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

class Path():
    def __init__(self, parent):
        self.parent = parent
        self.mode = 0
        self.cells = []
        self.box = 0
        self.assembled = [[],[],[]]
        print("[INFO] Starting vision system...")
        self.vision = Vision()
        print("[INFO] Vision system started.")
        self.disassembly = False

################################# PATH SCRIPTS #####################################################################
    def main_script(self):
        try:
            """Main script to execute the robot program."""
            # Set robot speed and acceleration
            self.parent.robot.setSpeed(50)
            self.parent.robot.setAcceleration(100)
            self.parent.robot.setSpeedJoints(15)
            self.parent.robot.setAccelerationJoints(30)


            #start_process = perf_counter()
            # If automatic assembly, use vision system to detect box and cells
            if self.mode == 1 or self.mode == 3:
                print("[INFO] Assembly starting...")
                # Call vision system to detect box and cells
                self.vision.start_up()
                self.vision.detect_box()
                if not self.vision.box_mold_detected:
                    print("[ERROR] Box not detected.")
                    self.box = 0
                    return
                self.vision.detect_lid()
                if not self.vision.lid_mold_detected:
                    print("[ERROR] Lid not detected.")
                    self.box = 0
                    return
                self.vision.detect_cell()
                if not self.vision.cell_mold_detected:
                    print("[ERROR] Cells not detected.")
                    self.cells = []
                    return
            
                #self.vision.view()
                # Check if box and lid match
                if self.mode == 1:
                    for detected in self.vision.box_mold_detected:
                        if detected in self.vision.lid_mold_detected:
                            self.box = detected
                            break
                        else:
                            self.box = 0
                    
                    if len(self.vision.cell_mold_detected[:4]) != 4:
                        print("[WARNING] Not enough cells detected")
                        self.cells = []
                        return
                    else:    
                        self.cells = self.vision.cell_mold_detected[:4]  # Get first 4 detected cells
                    
                elif self.mode == 3:
                    if self.box not in self.vision.box_mold_detected or self.box not in self.vision.lid_mold_detected:
                        print("[ERROR] Box or lid selected were not found.")
                        self.box = []
                        return
                
                if len(self.assembled[self.box-1]) != 0:
                    print(f"[WARNING] Selected box {self.box} has already been assembled with cells {self.assembled[self.box-1]}, please return the box and cells to their previous assembled state, or resart and reset the cell.")
                    return
                else:
                    print(f"[INFO] Box and lid correctly selected: {self.box}.")
                    
                for cell in self.cells:
                    if cell not in self.vision.cell_mold_detected:
                        print(f"[ERROR] Selected cell {cell} not detected.")
                        self.cells = []
                        return
                    for finished in self.assembled:
                        if cell in finished:
                            print(f"[ERROR] Cell {cell} is already part of an assembled box, please return the box and cells to their previous assembled state, or resart and reset the cell")
                            self.cells = []
                            return             

                # If no match found, return error
                if self.box == 0:
                    print("[ERROR] There are no matching box/lid.")
                    return     
                
            box = self.box      # Box number to assemble (1, 2, or 3)
            cells = self.cells  # Cells to place in the box (4 cells in total, from 1 to 12)

            if self.disassembly == False:
                # check if the box is already assembled
                # Perform assembly
                print(f"[INFO] Performing assembly of box {box} and cells {cells}")
                self.place_box(box)
                for i in range(1, 5):
                    self.place_cell(cells[i-1], i)
                # Keep track of the assembled box and its cells
                self.assembled[box-1] = cells
                self.place_full_box(box)
                self.place_lid(box)
                print(f"[INFO] Box assembled successfully with cells: {self.assembled[box-1]}.")
            elif self.disassembly == True:
                # check if the box is already assembled
                if len(self.assembled[box-1]) == 0:
                    print("[WARNING] Selected box has not been assembled.")
                    return
                else:
                    # Perform disassembly
                    print(f"[INFO] Performing disassembly of box {box} and cells {cells}")
                    cells = self.assembled[box-1]
                    self.disassemble_lid(box)
                    self.disassemble_full_box(box)
                    for i in range(1, 5):
                        self.disassemble_cell(cells[i-1], i)
                    # Empty the assembled box
                    self.assembled[box-1] = []
                    self.disassemble_box(box)
                    # Replace virtual box and its cells
                    self.parent.virtual_replace(box, cells)
                    
            
            print("[INFO] Main script execution complete.")
            #final_process = perf_counter()
            #execution_time = final_process - start_process 
            #print(f"Execution time : {execution_time}")
            self.running = False
        except Exception as e:
            print(f"[WARNING] {e}")
    
    def script_execution(self, targets):
        '''Execute the script with the given targets'''
        self.parent.RDK.setRunMode(RUNMODE_RUN_ROBOT)

        # Set reference and tool
        reference_frame = self.parent.RDK.Item('UR10e Base', ITEM_TYPE_FRAME)
        tool = self.parent.RDK.Item('tcp', ITEM_TYPE_TOOL)

        try:
            self.parent.robot.setPoseFrame(reference_frame)
            self.parent.robot.setPoseTool(tool)
        except:
            pass

        # Follow path
        for name in targets:
            target = self.parent.RDK.Item(name, ITEM_TYPE_TARGET)
            if not target.Valid():
                print(f"[WARNING] Target '{name}' not found!")
                continue
            else:
                self.parent.robot.MoveJ(target)

################################ ASSEMBLY AND DISASSEMBLY PATHS ####################################################
    def place_box(self, box_number):
        self.parent.gripper.gripper_open()
        targets = ['Home']

        #go to take a box
        targets.append(f"box{box_number}pos")
        targets.append(f"box{box_number}go")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_close_box(box_number)


        #take out the box and put it in the assembly station
        targets.append(f"box{box_number}up") 
        targets.append(f"box{box_number}out")
        targets.append('box_to_assembly')
        targets.append(f"box_assembly{box_number}approach")
        targets.append(f"box_assembly{box_number}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_open()
        targets.append('assembly_out')
        targets.append('assembly_up')
        self.script_execution(targets)
        targets = []
        
    def place_cell(self, cell_number, position_in_box):
        targets = ['Home']

        #go to take a cell
        targets.append(f"cell{cell_number}pos")
        targets.append(f"cell{cell_number}go")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_close_cell(cell_number)

        #take out the cell and put it in the box
        targets.append(f"cell{cell_number}up")
        targets.append(f"cell{cell_number}out")
        targets.append('Home')
        targets.append(f"place_cell{position_in_box}pos")
        targets.append(f"place_cell{position_in_box}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_open_cell() 
     
        targets.append(f"place_cell{position_in_box}pos")
        self.script_execution(targets)
        targets = []

    def place_full_box(self, shelf_position):
        self.parent.gripper.gripper_open()
        targets = ['Home']
        #go to take the full box
        targets.append('full_box_position') 
        targets.append('grab_full_box')
        self.script_execution(targets)
        self.parent.gripper.gripper_close_box(self.box)
        targets = []
        targets.append('grab_full_box_approach')
        
        #put full box in the shelf
        targets.append('full_box_up')
        targets.append('go_to_shelf') 
        targets.append(f"place{shelf_position}pos")
        targets.append(f"place{shelf_position}approach")
        targets.append(f"place{shelf_position}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_open()

        #change the grip of the lid
        targets.append(f"place{shelf_position}out")
        targets.append('Home')
        self.script_execution(targets)
        targets = []
        
    def place_lid(self, lid_number):   
        self.parent.gripper.gripper_open()
        targets = ['Home']
        
        #go to take a lid
        targets.append(f"lid{lid_number}pos")
        targets.append(f"lid{lid_number}go")
        self.script_execution(targets)
        self.parent.gripper.gripper_close_lid(lid_number)
        targets = []

        #take out the lid and put it in the assembly station
        targets.append(f"lid{lid_number}up")
        targets.append(f"lid{lid_number}out1")
        targets.append(f"lid{lid_number}out")
        targets.append('lid_to_assembly')
        targets.append('lid_in_assembly')
        targets.append(f"lid_assembly{lid_number}approach")
        targets.append(f"lid_assembly{lid_number}down")
        self.script_execution(targets)
        self.parent.gripper.gripper_open()
        targets = []

        #change the grip of the lid
        targets.append('new_grab')
        targets.append('new_grab_down')
        self.script_execution(targets)
        self.parent.gripper.gripper_close_lid(lid_number)
        targets = []

        #put lid on the box
        targets.append('new_grab_approach')
        targets.append('lid_to_shelf')
        targets.append(f"lid{lid_number}shelf")
        targets.append(f"lid{lid_number}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_open()


        #leave lid on the shelf
        targets.append(f"lid{lid_number}shelf")
        targets.append('Home')
        self.script_execution(targets)
        targets = []
    
    def disassemble_box(self, box_number):
        self.parent.gripper.gripper_open()
        targets = ['Home']
        #take box from assembly
        targets.append('assembly_up')
        targets.append('assembly_out')
        targets.append(f"box_assembly{box_number}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_close_box(box_number)
        
        #take box and put it in the shelf
        targets.append(f"box_assembly{box_number}approach")
        targets.append('box_to_assembly')
        targets.append(f"box{box_number}out")
        targets.append(f"box{box_number}up")
        targets.append(f"box{box_number}approach")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_open()

        #leave box
        targets.append(f"box{box_number}pos")
        targets.append('Home')
        self.script_execution(targets)
        targets = []  

    def disassemble_cell(self, cell_number, position_in_box):
        targets = ['Home']
        targets.append(f"place_cell{position_in_box}pos")
        targets.append(f"place_cell{position_in_box}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_close_cell(cell_number)
        
        #take out the cell and put it in the shelf
        targets.append(f"place_cell{position_in_box}pos")
        targets.append('Home')
        targets.append(f"cell{cell_number}out")
        targets.append(f"cell{cell_number}up")
        targets.append(f"cell{cell_number}go")
        self.script_execution(targets)
        self.parent.gripper.gripper_open_cell()
        targets = []

        #leave cell
        targets.append(f"cell{cell_number}pos")
        targets.append('Home')
        self.script_execution(targets)
        targets = []

    def disassemble_full_box(self, shelf_position):
        self.parent.gripper.gripper_open()
        targets = []

        #take box from final shelf
        targets.append(f"place{shelf_position}out")
        targets.append(f"place{shelf_position}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_close_box(shelf_position)
        
        #put full box in the shelf
        targets.append(f"place{shelf_position}approach")
        targets.append(f"place{shelf_position}pos")
        targets.append('go_to_shelf')
        targets.append('full_box_up')
        targets.append('grab_full_box_approach')
        targets.append('grab_full_box')
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_open()
        
        #leave full box in assembly
        targets.append('full_box_position')
        targets.append('Home')
        self.script_execution(targets)
        self.parent.gripper.gripper_open_cell()

    def disassemble_lid(self, lid_number):
        self.parent.gripper.gripper_open()

        #take lid from the shelf
        targets = ['Home']
        targets.append(f"lid{lid_number}shelf")
        targets.append(f"lid{lid_number}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_close_lid(lid_number)

        #put lid on the box
        targets.append(f"lid{lid_number}shelf")
        targets.append('lid_to_shelf')
        targets.append('new_grab')
        targets.append(f'new_grab{lid_number}approach')
        targets.append(f'new_grab_{lid_number}down')
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_open()
        
        #change the grip of the lid
        targets.append('new_grab')
        targets.append(f"lid_assembly{lid_number}down")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_close_lid(lid_number)

        #take out the from assembly and put it in the shelf
        targets.append(f"lid_assembly{lid_number}approach")
        targets.append('lid_in_assembly')
        targets.append('lid_to_assembly')
        targets.append(f"lid{lid_number}out")
        targets.append(f"lid{lid_number}out1")
        targets.append(f"lid{lid_number}up")
        targets.append(f"lid{lid_number}go")
        self.script_execution(targets)
        targets = []
        self.parent.gripper.gripper_open()
        #self.virtual_gripper_update(f"Detach")
        
        #leave lid
        targets.append(f"lid{lid_number}pos")
        targets.append('Home')
        self.script_execution(targets)
        targets = []

class Gripper():
    def __init__(self, parent):
        ''' Initialize the gripper. '''
        self.parent = parent
        self.RDK = parent.RDK
        self.robot = parent.robot
        self.gripper_open_command = None
        self.gripper_open_cell_command = None
        self.gripper_close_box_command = None
        self.gripper_close_cell_command = None
        self.gripper_close_lid_command = None
        self.moving_gripper = False
        self.socket = None
        self.virtual_gripper = self.RDK.Item('Gripper', ITEM_TYPE_ROBOT)
        self.gripper_setup()

    def gripper_setup(self):
        '''Sets up the gripper by reading the gripper commands from files,
        if the files are not found, it creates the programs from RoboDK.
        and finally connects to the socket.'''
        setup = False
        setup2 = False
        while not setup:
            try:
                # Read gripper commands from files
                with open(GRIPPER_OPEN_PATH, 'r') as f:
                    self.gripper_open_command = f.read()

                with open(GRIPPER_OPEN_CELL_PATH, 'r') as f:
                    self.gripper_open_cell_command = f.read()

                with open(GRIPPER_CLOSE_BOX_PATH, 'r') as f:
                    self.gripper_close_box_command = f.read()

                with open(GRIPPER_CLOSE_CELL_PATH, 'r') as f:
                    self.gripper_close_cell_command = f.read()
                
                with open(GRIPPER_CLOSE_LID_PATH, 'r') as f:
                    self.gripper_close_lid_command = f.read()
                
                setup = True
            except:
                # If files are not found, create the programs in RoboDK
                program = self.RDK.Item('gripper_open', ITEM_TYPE_PROGRAM)
                program.MakeProgram(GRIPPER_FOLDER_PATH)
                program = self.RDK.Item('gripper_open_cell', ITEM_TYPE_PROGRAM)
                program.MakeProgram(GRIPPER_FOLDER_PATH)
                program = self.RDK.Item('gripper_close_box', ITEM_TYPE_PROGRAM)
                program.MakeProgram(GRIPPER_FOLDER_PATH)
                program = self.RDK.Item('gripper_close_cell', ITEM_TYPE_PROGRAM)
                program.MakeProgram(GRIPPER_FOLDER_PATH)
                program = self.RDK.Item('gripper_close_lid', ITEM_TYPE_PROGRAM)
                program.MakeProgram(GRIPPER_FOLDER_PATH)
         # Connect to the gripper socket
        while not setup2:
            try:
                print(f"[INFO] Attempting to connect to the gripper at {IP}:{PORT}...")
                # Setup the socket connection for the gripper
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((IP, PORT))
                print(f"[INFO] Connected to gripper at {IP}:{PORT} successfully.")
                setup2 = True
            except (ConnectionRefusedError, TimeoutError, OSError) as e:
                print(f"[WARNING] Connection attempt failed: {e}")

    def gripper_open(self):
        '''Moves the virtual gripper to the open position,
        sends program to detach the virtual gripper from the robot
        and sends the command to open the gripper.'''
        if self.moving_gripper:
            print("[ERROR] Gripper is busy!")
            return
        self.moving_gripper = True
        # Set the run mode to simulate, to avoid executing the program on the real robot
        self.RDK.setRunMode(RUNMODE_SIMULATE)
        # Move the virtual gripper to the open position
        self.virtual_gripper.MoveJ([70])
        # Run the program to detach the gripper in the simulation
        prog = self.RDK.Item('Detach', ITEM_TYPE_PROGRAM)
        if prog.Valid():
            prog.RunCode()
        # Set the run mode back to run robot
        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        print("[INFO] Opening Gripper")
        # Send the command to open the gripper
        
        self.socket.send(self.gripper_open_command.encode('utf-8'))
        # Wait a second before reconnecting to ensure the command is sent correctly
        sleep(1)
        # Reconnect to the robot
        success = self.robot.Connect(IP)
        print("[INFO] Gripper Open")
        while not success:
            success = self.robot.Connect(IP)
            self.moving_gripper = True
        self.moving_gripper = False

    def gripper_open_cell(self):
        '''Moves the virtual gripper to the open position after cell,
        sends program to detach the virtual cell from the gripper
        and sends the command to open the gripper.'''
        if self.moving_gripper:
            print("[ERROR] Gripper is busy!")
            return
        self.moving_gripper = True
        # Set the run mode to simulate, to avoid executing the program on the real robot
        self.RDK.setRunMode(RUNMODE_SIMULATE)
        # Move the virtual gripper to the open position after cell
        self.virtual_gripper.MoveJ([38])
        # Run the program to detach the cell in the simulation
        prog = self.RDK.Item('Detach', ITEM_TYPE_PROGRAM)
        if prog.Valid():
            prog.RunCode()
            print("[INFO] Virtual cell detached")
        # Set the run mode back to run robot
        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        print("[INFO] Opening Gripper")
        # Send the command to open the gripper
        self.socket.send(self.gripper_open_cell_command.encode('utf-8'))
        # Wait a second before reconnecting to ensure the command is sent correctly
        sleep(1)
        # Reconnect to the robot
        success = self.robot.Connect(IP)
        print("[INFO] Gripper Open")
        while not success:
            success = self.robot.Connect(IP)
            self.moving_gripper = True
        self.moving_gripper = False

    def gripper_close_box(self, box_num):
        '''Moves the virtual gripper to the box position,
        sends program to attach the virtual box to the gripper
        and sends the command to close the gripper.'''
        if self.moving_gripper:
            print("[ERROR] Gripper is busy!")
            return
        self.moving_gripper = True
        # Set the run mode to simulate, to avoid executing the program on the real robot
        self.RDK.setRunMode(RUNMODE_SIMULATE)
        # Move the virtual gripper to the box position
        self.virtual_gripper.MoveJ([63])
        # Run the program to attach the box in the simulation
        prog = self.RDK.Item(f'Attach box{box_num}', ITEM_TYPE_PROGRAM)
        if prog.Valid():
            prog.RunCode()
            print("[INFO] Virtual box attached")
        # Set the run mode back to run robot
        # Keep track of the assembled box and its cells
        if self.parent.scripts.assembled[box_num-1] != 0:
            for i in self.parent.scripts.assembled[box_num-1]:
                prog = self.RDK.Item(f'Attach cell{i}', ITEM_TYPE_PROGRAM)
                if prog.Valid():
                    prog.RunCode()
                    print("[INFO] Virtual cell attached")
        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        print("[INFO] Gripping Box")
        # Send the command to close the gripper
        self.socket.send(self.gripper_close_box_command.encode('utf-8'))
        # Wait a second before reconnecting to ensure the command is sent correctly
        sleep(1)
        # Reconnect to the robot
        success = self.robot.Connect(IP)
        print("[INFO] Box Gripped")
        while not success:
            success = self.robot.Connect(IP)
            self.moving_gripper = True
        self.moving_gripper = False

    def gripper_close_cell(self, cell_num):
        '''Moves the virtual gripper to the cell position,
        sends program to attach the virtual cell to the gripper
        and sends the command to close the gripper.'''
        if self.moving_gripper:
            print("[ERROR] Gripper is busy!")
            return
        self.moving_gripper = True
        # Set the run mode to simulate, to avoid executing the program on the real robot
        self.RDK.setRunMode(RUNMODE_SIMULATE)
        # Move the virtual gripper to the cell position
        self.virtual_gripper.MoveJ([25])
        # Run the program to attach the cell in the simulation
        prog = self.RDK.Item(f'Attach cell{cell_num}', ITEM_TYPE_PROGRAM)
        if prog.Valid():
            prog.RunCode()
            print("[INFO] Virtual cell attached")
        # Set the run mode back to run robot
        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        print("[INFO] Gripping cell")
        # Send the command to close the gripper
        self.socket.send(self.gripper_close_cell_command.encode('utf-8'))
        # Wait a second before reconnecting to ensure the command is sent correctly
        sleep(1)
        # Reconnect to the robot
        success = self.robot.Connect(IP)
        print("[INFO] cell Gripped")
        while not success:
            success = self.robot.Connect(IP)
            self.moving_gripper = True
        self.moving_gripper = False

    def gripper_close_lid(self, lid_num):
        '''Moves the virtual gripper to the lid position,
        sends program to attach the virtual lid to the gripper
        and sends the command to close the gripper.'''
        if self.moving_gripper:
            print("[ERROR] Gripper is busy!")
            return
        self.moving_gripper = True
        # Set the run mode to simulate, to avoid executing the program on the real robot
        self.RDK.setRunMode(RUNMODE_SIMULATE)
        # Move the virtual gripper to the lid position
        self.virtual_gripper.MoveJ([17])
        # Run the program to attach the lid in the simulation
        prog = self.RDK.Item(f'Attach lid{lid_num}', ITEM_TYPE_PROGRAM)
        if prog.Valid():
            prog.RunCode()
            print("[INFO] Virtual lid attached")
        # Set the run mode back to run robot
        self.RDK.setRunMode(RUNMODE_RUN_ROBOT)
        print("[INFO] Gripping Lid")
        # Send the command to close the gripper
        self.socket.send(self.gripper_close_lid_command.encode('utf-8'))
        # Wait a second before reconnecting to ensure the command is sent correctly
        sleep(1)
        # Reconnect to the robot
        success = self.robot.Connect(IP)
        print("[INFO] Lid Gripped")
        while not success:
            success = self.robot.Connect(IP)
            self.moving_gripper = True
        self.moving_gripper = False


class Robot():
    def __init__(self, robot, rdk):
        super().__init__()

        self.robot = robot
        self.RDK = rdk  # Store RoboDK instance

        self.gripper = Gripper(self)  # Initialize the gripper

        self.connect() # Connect to the robot
        
        self.scripts = Path(self)

################################# ROBOT CONNECTION AND MOVEMENT ###################################################
    def connect(self):
        if self.robot.Connect(IP):
            print("Robot connected.")
        else:
            print("Robot connection failed.")
                        
    def execute(self):
        """Executes the predefined robot program once."""
        if self.robot.Busy():
            print("[ERROR] Robot is busy!")
            return
        print("[INFO] Executing SCRIPT...")


        # Ask for mode selection
        self.scripts.mode = int(input("Select mode: 1 - Automatic assembly, 2 - Automatic disassembly, 3 - Manual assembly: "))
        if self.scripts.mode == 1:      # Automatic assembly
            self.scripts.disassembly = False
        elif self.scripts.mode == 2:    # Automatic disassembly
            self.scripts.disassembly = True
            # Select box for automatic disassembly
            self.scripts.box = int(input("Select box (1, 2, 3): "))
        elif self.scripts.mode == 3:    # Manual assembly
            self.scripts.disassembly = False
            # Select box and cells for manual assembly
            self.scripts.box = int(input("Select box (1, 2, 3): "))
            self.scripts.cells = []
            for i in range(1, 5):
                self.scripts.cells.append(int(input("Select cell (1 - 12): "))) 
        else:
            print("[ERROR] Invalid mode selected.")
            return
        # Start the script in a separate thread
        try:
            self.scripts.main_script()
        except:
            print("[ERROR] There was a problem executing the program, please try again.")
        print("[INFO] Program sent for execution.")

###################################################################################################################
#################### MAIN SETUP AND EXECUTION #####################################################################
###################################################################################################################
def RunUI():
    ## Initialize RoboDK and the robot
    RDK = Robolink()
    try:
        robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
        if not robot.Valid():
            print("Error: Robot not found in RoboDK!")
            raise FileNotFoundError

        rob = Robot(robot, RDK)
        while input("Run code? y/n: ") == 'y':
            rob.execute()
    except FileNotFoundError:
        print("Opening station...")
        RDK.AddFile(ROBOTIC_STATION_PATH)
        robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
        if not robot.Valid():
            print("Error: Robot not found in RoboDK!")
            raise SystemError

        rob = Robot(robot, RDK)
        while input("Run code? y/n: ") == 'y':
            rob.execute()

# Run the UI externally
if __name__ == "__main__":
    RunUI()
