#!/usr/bin/env python
import mediapipe as mp
import cv2
import rospy
from std_msgs.msg import Int16
import time

pub = rospy.Publisher('gesture', Int16, queue_size=10)
rospy.init_node('gesturePub')
    

class gestureRecognition:
    def __init__(self) -> None:
        self.mpDraw = mp.solutions.drawing_utils
        self.drawStyles = mp.solutions.drawing_styles
        self.hand = mp.solutions.hands.Hands(model_complexity=0)
        self.baseOptions = mp.tasks.BaseOptions
        self.gestureRecognizer = mp.tasks.vision.GestureRecognizer
        self.gestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        self.gestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
        self.runningMode = mp.tasks.vision.RunningMode
        self.options = self.gestureRecognizerOptions(
                base_options = self.baseOptions(model_asset_path = 'gesture_recognizer.task', delegate = "GPU"),
                running_mode = self.runningMode.IMAGE,
                min_hand_detection_confidence = 0.4,
                canned_gesture_classifier_options = mp.tasks.components.processors.ClassifierOptions(score_threshold = 0.7))

    def drawLandmarks(self, img) -> None:
        imgForProcess = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        self.result = self.hand.process(imgForProcess)
        self.lm = self.result.multi_hand_landmarks
        if (self.lm):
            for hand in self.lm:
                self.mpDraw.draw_landmarks(
                        img,
                        hand,
                        mp.solutions.hands.HAND_CONNECTIONS,
                )

    def recognizeGesture(self, imgFromOpenCV):
        with self.gestureRecognizer.create_from_options(self.options) as recognizer:
            mpImage = mp.Image(image_format = mp.ImageFormat.SRGB, data = imgFromOpenCV)
            result = recognizer.recognize(mpImage)
            if (result.gestures):
                return result.gestures[0][0].category_name

def main() -> None:
    while not rospy.is_shutdown():
        flag = False
        isTrueFirstTime = False
        vid = cv2.VideoCapture(0)
        recognizerInstance = gestureRecognition()
        while True:
            ret, frame = vid.read()
            if not ret:
                continue
            if (flag is True):
                if (isTrueFirstTime):
                    startTime = time.time()
                    isTrueFirstTime = False
                currTime = time.time()
                duration = currTime - startTime
                if (duration < 3):
                    continue
                else:
                    flag = False
            gesture = recognizerInstance.recognizeGesture(frame)
            print(gesture)
            if (gesture == "Closed_Fist"):
                pub.publish(1)
                flag = True
                isTrueFirstTime = True
            elif (gesture == "Victory"):
                pub.publish(2)
                flag = True
                isTrueFirstTime = True
            elif (gesture == "Thumb_Up"):
                pub.publish(3)
                isTrueFirstTime = True
                flag = True
            elif (gesture == "Pointing_Up"):
                pub.publish(4)
                flag = True
                isTrueFirstTime = True
            cv2.imshow("Video", frame)
            k = cv2.waitKey(5)
            if (k==ord("q")):
                break
        
if __name__ == "__main__":
    main()
