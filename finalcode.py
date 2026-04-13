import cv2
import torch
from robomaster import robot, camera
import sys
import time

# Initialize the variables (All members)
distance = []
angle = 270 # Starting angle
coordx = 0
coordy = 0
grabbing = False
searching = True
running = True
confidence_score = 0.3 # Confidence score threshold, to prevent wrong detections

# Load the YOLOv5 model (All members)
sys.path.insert(0, './yolov5')
model_path = r'C:\Users\angus\PycharmProjects\RoboMaster-SDK-master\yolov5\realfinalbest.pt' #final model
model_path2 = r'C:\Users\angus\PycharmProjects\RoboMaster-SDK-master\yolov5'
model = torch.hub.load(model_path2, 'custom', path=model_path, source='local')

# Connect and initialize the robot (All members)
rm = robot.Robot()
rm.initialize(conn_type="ap")
arm = rm.robotic_arm
chassis = rm.chassis
sensor = rm.sensor
gripper = rm.gripper

# Open the gripper (All members)
gripper.open(power=200)

# Define the class labels for the recyclable garbages (All members)
class_labels = ['paper', 'metal', 'plastic','finish'] #when the class label is 'finished', all garbages are picked up
current_index = 0

# IR sensor (All members)
def sub_data_handler(sub_info):
    global distance
    distance = sub_info

# Robomaster coordinate system (All members)
def sub_position_handler(position_info):
    global angle, coordx, coordy
    coordx, coordy, z = position_info

# Calculate the center of the yolo rectangle (All members)
def center(max, min):
    return (int(max)+int(min))/2

# Main loop to capture images and detect objects (All members)
rm.camera.start_video_stream(display=True, resolution=camera.STREAM_360P)

# Call the IR sensor and coordinate system funtcion (runs parallel with the while loop) (All members)
sensor.sub_distance(freq=1, callback=sub_data_handler) # Lower frequecny seems to make the code smoother
chassis.sub_position(freq=1, callback=sub_position_handler) # Lower frequecny seems to make the code smoother

# Initial position and orientation (All members)
chassis.move(x=0.2, y=0, z=0, xy_speed=0.3).wait_for_completed()
chassis.move(x=0, y=0, z=90, z_speed=45).wait_for_completed()

# Main loop
while running:

    # Searching mode (1st step)
    if searching:

        # Initialize the rotation counter (Chan Angus)
        rotatetime = 0
        rotatez = 30
        print('searching')
        search_rotate = True

        # Rotate while searching
        while search_rotate:

            # Rotated 180 degree and cannot find the selected garbages, it means that all garbages of the specific class are picked up and we can move on to the next garbage
            if rotatetime == 6:
                current_index += 1
                rotatetime = 0
                rotatez = -rotatez

            # All 3 classes of garbages are picked up and the whole code is finished
            if class_labels[current_index] == 'finish':
                print('finished')
                running=False
                break

            # Capture an image from the camera (Hui Lap Pong)
            image = rm.camera.read_cv2_image()
            image = cv2.resize(image, (640, 480))

            # Perform object detection with the YOLOv5 model
            outputs = model(image)
            detections = outputs.pandas().xyxy[0]
            print('rotating')

            # Detection with yolov5 model
            for i, detection in detections.iterrows():

                # If garbages with specific class is detected, move on to the grabbing mode
                if detection['name'] in [class_labels[current_index]] and detection['confidence'] >= confidence_score:
                    print('detected '+str(detection['name']))
                    searching = False
                    grabbing = True
                    search_rotate = False

            # Rotate 30 degree everytime during searching mode
            if searching:
                chassis.move(x=0, y=0, z=-rotatez, z_speed=45).wait_for_completed()
                rotatetime += 1
                angle += rotatez

            # Angle calculations
            if angle > 360:
                angle -= 360
            elif angle < 0:
                angle += 360

    # Grabbing mode (2nd step) (Seonhyeok)
    while grabbing:

        # Capture an image from the camera
        image = rm.camera.read_cv2_image()
        image = cv2.resize(image, (640, 480))

        # Perform object detection with the YOLOv5 model
        outputs = model(image)
        detections = outputs.pandas().xyxy[0]
        print(detections)

        # Process the detections
        if len(detections) > 0:
            for i, detection in detections.iterrows():
                # Start centering the garbage when detected
                if detection['name'] in [class_labels[current_index]] and detection['confidence'] >= confidence_score:
                    center_x, center_y = center(detection['xmax'], detection['xmin']), center(detection['ymax'], detection['ymin'])

                    # The garbage is on the left side on the center of the screen, the robot will move left to center the garbage
                    if center_x <= 317:
                        print('moving left')
                        chassis.drive_speed(x=0, y=-0.05, z=0, timeout=1)

                    # The garbage is on the right side on the center of the screen, the robot will move right to center the garbage
                    elif center_x >= 322:
                        print('moving right')
                        chassis.drive_speed(x=0, y=0.05, z=0, timeout=1)

                    # The garbage is located at the center of the screen, the robot can move forward to grab the garbage
                    elif center_x > 317 and center_x < 322:

                        # Move forward a bit first, so that the robot can still grab the garbage if it is too close
                        print('going forward...')
                        chassis.drive_speed(x=0.15, y=0, z=0, timeout=1)
                        time.sleep(1)

                        # Moving towards the garbage (Chan Angus)
                        while True:

                            # Keep moving forward if the robot have not reached the garbage
                            chassis.drive_speed(x=0.15, y=0, z=0, timeout=1)

                            # Reached the garbage
                            if distance[0] <= 30:
                                print('reached the object')
                                chassis.drive_speed(x=0, y=0, z=0, timeout=1) # Stop the robot
                                print('closing')
                                time.sleep(1)
                                gripper.close(power=150) # Close the gripper
                                time.sleep(1)
                                print('raising')
                                arm.move(x=0, y=90).wait_for_completed() # Raise the gripper
                                time.sleep(1)
                                tempx, tempy = coordx, coordy # Record the coordinates of the robot
                                print(tempx, tempy)

                                # Grab the garbages to specific locations (Hui Chung Hin)
                                if current_index == 2:
                                    chassis.move(x=0, y=0, z=-(90-angle), z_speed=90).wait_for_completed() # Points towards the plastic zone
                                    angle = 180
                                    print('moving' + str(1-tempy))
                                    chassis.move(x=1 - tempy, y=0, z=0, xy_speed=0.7).wait_for_completed()  # Moves to the plastic zone
                                elif current_index == 1:
                                    chassis.move(x=0, y=0, z=-(360-angle), z_speed=90).wait_for_completed() # Points towards the metal zone
                                    angle = 90
                                    print('moving' + str(2-tempx))
                                    chassis.move(x=2 - tempx, y=0, z=0, xy_speed=0.7).wait_for_completed()  # Moves to the metal zone
                                elif current_index == 0:
                                    chassis.move(x=0, y=0, z=-(270-angle), z_speed=90).wait_for_completed() # Points towards the paper zone
                                    angle = 0
                                    print('moving' + str(1+tempy))
                                    chassis.move(x=1 + tempy, y=0, z=0, xy_speed=0.7).wait_for_completed()  # Moves to the paper zone

                                time.sleep(2) #(Seonhyeok)
                                arm.move(x=0, y=-90).wait_for_completed() # Lower the gripper
                                print('lowering')
                                gripper.open(power=300) # Open the gripper
                                print('open')
                                time.sleep(1)
                                gripper.pause()
                                chassis.move(x=-0.2, y=0, z=0, xy_speed=0.1).wait_for_completed() # Move backwards
                                chassis.move(x=0, y=0, z=-90, z_speed=90).wait_for_completed()  # rotate to search position
                                print('moving out')

                                # Goes back to searching mode
                                searching = True
                                grabbing = False
                                print('end')
                                break
                    break

        # Exit the program
        if cv2.waitKey(1) > 0:
            break

# Release the camera and disconnect from the robot
rm.close()
cv2.destroyAllWindows()
rm.camera.stop_video_stream()


