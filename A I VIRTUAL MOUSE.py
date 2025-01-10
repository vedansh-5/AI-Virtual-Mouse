import cv2                # perform image processing
import mediapipe as mp    # perform hand detection
import pyautogui          # control the mouse and keyboard tasks is system
import math               # In the Machine Learning algorithms we use mathematical concepts
from enum import IntEnum  # Intenum is Base class of enum and we get integer values through it
from ctypes import cast, POINTER  # Its a foreign function library and provides C compatible data types, and allows calling functions in DLLs or shared libraries.
from comtypes import CLSCTX_ALL   # deals with creating a large extent of class objects
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume  # use for changing volume up down level
from google.protobuf.json_format import MessageToDict  # Return a string containing the JSON formatted protocol buffer message
import screen_brightness_control as sbcontrol  # used for controlling the  screen brightness level

'''       Hand Tracking
          /          \
         /            \
Palm detection    Hand Landmarks   '''

pyautogui.FAILSAFE = False   # This feature stop execution of your pyautogui program by manually moving the mouse to the upper left corner of the screen.It is basically a exception handling.
mp_drawing = mp.solutions.drawing_utils   # Used for creating landmarks on hand (the lines and points shown on hands is because of this method )
mp_hands = mp.solutions.hands             # The above one and this both are methods of handtracking module

# Gesture Encodings
class Gest(IntEnum):
    # Binary Encoded
    FIST = 0        # we have the 21 hand points that MediaPipe identifies for single hand 
    PINKY = 1       # and for both the hands we have the 42 hand points
    RING = 2
    MID = 4
    LAST3 = 7
    INDEX = 8
    FIRST2 = 12
    LAST4 = 15
    THUMB = 16    
    PALM = 31
    
    # Extra Mappings
    V_GEST = 33
    TWO_FINGER_CLOSED = 34
    PINCH_MAJOR = 35      # Pinch of major hand->right hand
    PINCH_MINOR = 36       # Pinch of minor hand->left hand

# Multi-handedness Labels
class HLabel(IntEnum):  # for more then one hands in a frame
    MINOR = 0           # Maxmium two hands will be detected at a time in frame
    MAJOR = 1           # To detect more then one hand in a frame just change value from '=2' in line 366


# Convert Mediapipe Landmarks to recognizable Gestures
class HandRecog:
                                     # Creating constructor under class for initializing an object’s state
    def __init__(self, hand_label):  # _init is called whenever a class object is created.
        self.finger = 0              # self represents a class instance and is required to access any variables or methods within the class.
        self.ori_gesture = Gest.PALM
        self.prev_gesture = Gest.PALM     # Initialization of parameters of class
        self.frame_count = 0
        self.hand_result = None
        self.hand_label = hand_label
    
    def update_hand_result(self, hand_result):
        self.hand_result = hand_result

    def get_signed_dist(self, point):         # creating the hand landmarks.......
        sign = -1
        if self.hand_result.landmark[point[0]].y < self.hand_result.landmark[point[1]].y: # Creating the x and y coordinates on hands for image processing because in opencv image is read in the form of array.
            sign = 1.
        dist = (self.hand_result.landmark[point[0]].x - self.hand_result.landmark[point[1]].x)**2
        dist = (self.hand_result.landmark[point[0]].y - self.hand_result.landmark[point[1]].y)**2
        dist = math.sqrt(dist)    # math library is used for the mathematical computations..for eg square , multiplication , power operations.
        return dist*sign
    
    def get_dist(self, point):
        dist = (self.hand_result.landmark[point[0]].x - self.hand_result.landmark[point[1]].x)**2
        dist += (self.hand_result.landmark[point[0]].y - self.hand_result.landmark[point[1]].y)**2
        dist = math.sqrt(dist)
        return dist
    
    def get_dz(self,point):
        return abs(self.hand_result.landmark[point[0]].z - self.hand_result.landmark[point[1]].z)
    
    # Function to find Gesture Encoding using current finger_state.
    # Finger_state: 1 if finger is open, else 0
    def set_finger_state(self):
        if self.hand_result == None:
            return

        points = [[8,5,0],[12,9,0],[16,13,0],[20,17,0]]
        self.finger = 0
        self.finger = self.finger | 0       #thumb
        for idx,point in enumerate(points):
            
            dist = self.get_signed_dist(point[:2])
            dist2 = self.get_signed_dist(point[1:])
            
            try:       # Python exception handling 
                ratio = round(dist/dist2,1)
            except:
                ratio = round(dist2/0.01,1)

            self.finger = self.finger << 1
            if ratio > 0.5 :
                self.finger = self.finger | 1
    

    # Handling Fluctations due to noise
    def get_gesture(self):
        if self.hand_result == None:
            return Gest.PALM

        current_gesture = Gest.PALM
        if self.finger in [Gest.LAST3,Gest.LAST4] and self.get_dist([8,4]) < 0.05:
            if self.hand_label == HLabel.MINOR :
                current_gesture = Gest.PINCH_MINOR
            else:
                current_gesture = Gest.PINCH_MAJOR

        elif Gest.FIRST2 == self.finger :
            point = [[8,12],[5,9]]
            dist1 = self.get_dist(point[0])
            dist2 = self.get_dist(point[1])
            ratio = dist1/dist2
            if ratio > 1.7:
                current_gesture = Gest.V_GEST
            else:
                if self.get_dz([8,12]) < 0.1:
                    current_gesture =  Gest.TWO_FINGER_CLOSED
                else:
                    current_gesture =  Gest.MID
            
        else:
            current_gesture =  self.finger
        
        if current_gesture == self.prev_gesture:
            self.frame_count += 1
        else:
            self.frame_count = 0

        self.prev_gesture = current_gesture

        if self.frame_count > 4 :
            self.ori_gesture = current_gesture
        return self.ori_gesture

# Executes commands according to detected gestures
class Controller:
    tx_old = 0
    ty_old = 0
    trial = True
    flag = False        # Initially setting the all flags to false later we reset them below in the code... 
    grabflag = False
    pinchmajorflag = False
    pinchminorflag = False
    pinchstartxcoord = None
    pinchstartycoord = None
    pinchdirectionflag = None
    prevpinchlv = 0
    pinchlv = 0
    framecount = 0
    prev_hand = None
    pinch_threshold = 0.3
    
    def getpinchylv(hand_result):
        dist = round((Controller.pinchstartycoord - hand_result.landmark[8].y)*10,1)
        return dist

    def getpinchxlv(hand_result):
        dist = round((hand_result.landmark[8].x - Controller.pinchstartxcoord)*10,1)
        return dist
    
    def changesystembrightness():    # Function to control brightness level through hand Gesture
        currentBrightnessLv = sbcontrol.get_brightness()/100.0
        currentBrightnessLv += Controller.pinchlv/50.0
        if currentBrightnessLv > 1.0:   # lowest Brightness level 
            currentBrightnessLv = 1.0
        elif currentBrightnessLv < 0.0:   # level of brightness vary between 0 to 100
            currentBrightnessLv = 0.0       
        sbcontrol.fade_brightness(int(100*currentBrightnessLv) , start = sbcontrol.get_brightness())
    
    def changesystemvolume():     # Function to control Volume level through hand Gesture
        devices = AudioUtilities.GetSpeakers()
        interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        volume = cast(interface, POINTER(IAudioEndpointVolume))
        currentVolumeLv = volume.GetMasterVolumeLevelScalar()
        currentVolumeLv += Controller.pinchlv/50.0
        if currentVolumeLv > 1.0:    # lowest volume level 
            currentVolumeLv = 1.0
        elif currentVolumeLv < 0.0:   # level of volume vary between 0 to 100
            currentVolumeLv = 0.0
        volume.SetMasterVolumeLevelScalar(currentVolumeLv, None)
    
    def scrollVertical():    # Function to Perform up/down scroll operation when scrollbar is in vertical mode means at the right side of the screen
        pyautogui.scroll(120 if Controller.pinchlv>0.0 else -120)
                               # -120 indicates scroll down upto 120 clicks
                               # 120 indicates scroll up upto 120 clicks 
    def scrollHorizontal():    # Function to Perform left/right scroll operation when scrollbar is at bottom of the screen
        pyautogui.keyDown('shift')
        pyautogui.keyDown('ctrl')
        pyautogui.scroll(-120 if Controller.pinchlv>0.0 else 120)
        pyautogui.keyUp('ctrl')
        pyautogui.keyUp('shift')
    
    # Locate Hand to get Cursor Position
    # Stabilize cursor by Dampening
    def get_position(hand_result):
        point = 9
        position = [hand_result.landmark[point].x ,hand_result.landmark[point].y]
        sx,sy = pyautogui.size()
        x_old,y_old = pyautogui.position()
        x = int(position[0]*sx)        # We get the values of x and y points in decimal so we convert
        y = int(position[1]*sy)        # them into integer values.
        if Controller.prev_hand is None:
            Controller.prev_hand = x,y
        delta_x = x - Controller.prev_hand[0]
        delta_y = y - Controller.prev_hand[1]

        distsq = delta_x**2 + delta_y**2
        ratio = 1
        Controller.prev_hand = [x,y]

        if distsq <= 25:
            ratio = 0
        elif distsq <= 900:
            ratio = 0.07 * (distsq ** (1/2))
        else:
            ratio = 2.1
        x , y = x_old + delta_x*ratio , y_old + delta_y*ratio
        return (x,y)

    def pinch_control_init(hand_result):
        Controller.pinchstartxcoord = hand_result.landmark[8].x
        Controller.pinchstartycoord = hand_result.landmark[8].y
        Controller.pinchlv = 0
        Controller.prevpinchlv = 0
        Controller.framecount = 0

    # Hold final position for 5 frames to change status
    def pinch_control(hand_result, controlHorizontal, controlVertical):
        if Controller.framecount == 5:
            Controller.framecount = 0
            Controller.pinchlv = Controller.prevpinchlv

            if Controller.pinchdirectionflag == True:
                controlHorizontal() #x           # Horizontal movement of hand pinch (x -> for right hand)

            elif Controller.pinchdirectionflag == False:
                controlVertical() #y             # Vertical movement of hand pinch (y -> for left hand)

        lvx =  Controller.getpinchxlv(hand_result)   # right hand
        lvy =  Controller.getpinchylv(hand_result)   # left hand
            
        if abs(lvy) > abs(lvx) and abs(lvy) > Controller.pinch_threshold:
            Controller.pinchdirectionflag = False
            if abs(Controller.prevpinchlv - lvy) < Controller.pinch_threshold:
                Controller.framecount += 1
            else:
                Controller.prevpinchlv = lvy
                Controller.framecount = 0

        elif abs(lvx) > Controller.pinch_threshold:
            Controller.pinchdirectionflag = True
            if abs(Controller.prevpinchlv - lvx) < Controller.pinch_threshold:
                Controller.framecount += 1
            else:
                Controller.prevpinchlv = lvx
                Controller.framecount = 0

    def handle_controls(gesture, hand_result):        
        x,y = None,None                 # Closed Palm Gesture to do not perform and operation..
        if gesture != Gest.PALM :
            x,y = Controller.get_position(hand_result)
        
        # flag reset
        if gesture != Gest.FIST and Controller.grabflag:
            Controller.grabflag = False
            pyautogui.mouseUp(button = "left")

        if gesture != Gest.PINCH_MAJOR and Controller.pinchmajorflag:
            Controller.pinchmajorflag = False

        if gesture != Gest.PINCH_MINOR and Controller.pinchminorflag:
            Controller.pinchminorflag = False

        # implementation
        if gesture == Gest.V_GEST:     # Gesture -> V shape of Index and Middle fingers to move the mouse cursior on the screen
            Controller.flag = True
            pyautogui.moveTo(x, y, duration = 0.1)

        elif gesture == Gest.FIST:    # Closing Fist Gesture to perform the drag and drop operation
            if not Controller.grabflag : 
                Controller.grabflag = True
                pyautogui.mouseDown(button = "left")
            pyautogui.moveTo(x, y, duration = 0.1)

        elif gesture == Gest.MID and Controller.flag:   # Right hand Middle finger gesture for performing Right click opeation
            pyautogui.click()
            Controller.flag = False

        elif gesture == Gest.INDEX and Controller.flag:  # Right hand Index finger gesture for performing left click opeation
            pyautogui.click(button='right')
            Controller.flag = False

        elif gesture == Gest.TWO_FINGER_CLOSED and Controller.flag:   # Gesture for closing two finger to perfom select or say double click operation.
            pyautogui.doubleClick()
            Controller.flag = False

        elif gesture == Gest.PINCH_MINOR:             # Pinch Gesture of Minor-hand(left hand) to perform opration 
            if Controller.pinchminorflag == False:    # Using left hand pinch gesture scroll operation is performed 
                Controller.pinch_control_init(hand_result)
                Controller.pinchminorflag = True
            Controller.pinch_control(hand_result,Controller.scrollHorizontal, Controller.scrollVertical)
        
        elif gesture == Gest.PINCH_MAJOR:          # Pinch Gesture of Major-hand(right hand) to perform opration 
            if Controller.pinchmajorflag == False: # Using right hand pinch gesture volume and brightness operation are performed 
                Controller.pinch_control_init(hand_result)
                Controller.pinchmajorflag = True
            Controller.pinch_control(hand_result,Controller.changesystembrightness, Controller.changesystemvolume)
        
'''
----------------------------------------  Main Class  ----------------------------------------
    Entry point of Gesture Controller
'''


class GestureController:
    gc_mode = 0
    cap = None
    CAM_HEIGHT = None
    CAM_WIDTH = None
    hr_major = None # Right Hand by default
    hr_minor = None # Left hand by default
    dom_hand = True

    def __init__(self):
        GestureController.gc_mode = 1
        GestureController.cap = cv2.VideoCapture(0)    #Capturing video through webcam of system
        GestureController.CAM_HEIGHT = GestureController.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        GestureController.CAM_WIDTH = GestureController.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    
    def classify_hands(results):    # Function for Controlling left and right hand functionality..........
        left , right = None,None
        try:
            handedness_dict = MessageToDict(results.multi_handedness[0])
            if handedness_dict['classification'][0]['label'] == 'Right':
                right = results.multi_hand_landmarks[0]
            else :
                left = results.multi_hand_landmarks[0]
        except:
            pass

        try:
            handedness_dict = MessageToDict(results.multi_handedness[1])
            if handedness_dict['classification'][0]['label'] == 'Right':
                right = results.multi_hand_landmarks[1]
            else :
                left = results.multi_hand_landmarks[1]
        except:
            pass
        
        if GestureController.dom_hand == True:  # We can change our major (Right hand) and minor (Left Hand) functionality here.
            GestureController.hr_major = right  # To perform functionality through left hand just change  
            GestureController.hr_minor = left   # the right -> left and left -> right in 359 to 364 lines
        else :
            GestureController.hr_major = left
            GestureController.hr_minor = right

    def start(self):              # Function for Controlling left and right hand functionality.........
        
        handmajor = HandRecog(HLabel.MAJOR)
        handminor = HandRecog(HLabel.MINOR)
                                               # Maximum 2 hands will be detected in frame
        with mp_hands.Hands(max_num_hands = 1,min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
            while GestureController.cap.isOpened() and GestureController.gc_mode:  # checking webcam is opened or not
                success, image = GestureController.cap.read()   # Reading the Frames of Webcam Video


                if not success:
                    print("Ignoring empty camera frame.")
                    continue
                
                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)  # Converting image Bgr to Rgb format
                image.flags.writeable = False   # setting false to stop for copying any other image before processing
                results = hands.process(image)
                
                image.flags.writeable = True  # setting true to get the results after image processing
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                if results.multi_hand_landmarks:                   
                    GestureController.classify_hands(results)
                    handmajor.update_hand_result(GestureController.hr_major)
                    handminor.update_hand_result(GestureController.hr_minor)

                    handmajor.set_finger_state()
                    handminor.set_finger_state()
                    gest_name = handminor.get_gesture()

                    if gest_name == Gest.PINCH_MINOR:
                        Controller.handle_controls(gest_name, handminor.hand_result)
                    else:
                        gest_name = handmajor.get_gesture()
                        Controller.handle_controls(gest_name, handmajor.hand_result)
                    
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                else:
                    Controller.prev_hand = None
                cv2.imshow('Gesture Controller', image)    # imshow() function to show reading image of webcam
                if cv2.waitKey(5) & 0xFF == 27:   # waitkey initialization on 'esc' button and 0xFF is the mask 
                    break                  # here in above line (5 time unit) denotes basically the speed of frames read through webcam and to increase speed of frame reading just decrease the value from 5.
        GestureController.cap.release()   # it closes video file or capturing device eg 'webcam'
        cv2.destroyAllWindows()    # used to free all memeory space 

# uncomment to run directly
if __name__ == "__main__":
    gc1 = GestureController()   # calling....
    gc1.start()