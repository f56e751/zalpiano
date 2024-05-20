import cv2
import numpy as np

class ColorTracker:
    def __init__(self, color_name, lower_bound, upper_bound, kalman_params):
        self.color_name = color_name
        self.lower_bound = np.array(lower_bound, dtype=int)
        self.upper_bound = np.array(upper_bound, dtype=int)
        self.kalman_filter = self.initialize_kalman(kalman_params)
        self.position = (0, 0)
        self.velocity = (0, 0)
        self.speed = 0
        self.direction = 0

        # self.calibration_state = None
        self.calibration_state = 'green'
        self.box_size = 50

    def initialize_kalman(self, kalman_params):
        kf = cv2.KalmanFilter(4, 2)  # 4 dynamic parameters (x, y, vx, vy), 2 measured parameters (x, y)
        kf.transitionMatrix = np.array(kalman_params['transitionMatrix'], np.float32)
        kf.measurementMatrix = np.array(kalman_params['measurementMatrix'], np.float32)
        kf.processNoiseCov = np.eye(4, dtype=np.float32) * kalman_params['processNoise']
        kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * kalman_params['measurementNoise']
        kf.errorCovPost = np.eye(4, dtype=np.float32) * kalman_params['errorCovPost']
        return kf

    def update(self, frame, hsv):
        mask = cv2.inRange(hsv, self.lower_bound, self.upper_bound)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 150:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cx, cy = x + w // 2, y + h // 2
                measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
                self.kalman_filter.correct(measurement)
                prediction = self.kalman_filter.predict()
                pos_x, pos_y, vel_x, vel_y = prediction.flatten()


                self.position = [pos_x,pos_y]
                # self.position = (prediction[0], prediction[1])
                # vel_x, vel_y = prediction[2], prediction[3]
                self.speed = np.sqrt(vel_x**2 + vel_y**2)
                self.velocity = (vel_x, vel_y)
                self.direction = np.degrees(np.arctan2(vel_y, vel_x))

    # def preCalibrateState(self,green,blue,yellow):
    #     self.

    def calibrate(self, frame):
        if self.calibration_state is None:
            return  # Calibration not started or already finished

        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        calibration_box = (center_x - self.box_size // 2, center_y - self.box_size // 2, self.box_size, self.box_size)
        
        color_map = {'green': (0, 255, 0), 'blue': (255, 0, 0), 'yellow': (0, 255, 255)}
        cv2.rectangle(frame, (calibration_box[0], calibration_box[1]),
                      (calibration_box[0] + calibration_box[2], calibration_box[1] + calibration_box[3]),
                      color_map[self.calibration_state], 2)  # Highlight the calibration area


        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.set_color_bounds(hsv, calibration_box)

    def preCalibrate(self,average_color):
        sensitivity = 10
        lower_bound = np.clip(average_color - np.array([sensitivity, 50, 50]), 0, 255)
        upper_bound = np.clip(average_color + np.array([sensitivity, 255, 255]), 0, 255)
        
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound


    def set_color_bounds(self, hsv, calibration_box):
        x, y, w, h = calibration_box
        roi = hsv[y:y+h, x:x+w]
        average_color = np.mean(roi.reshape(-1, 3), axis=0).astype(int)
        print(average_color)
        sensitivity = 10
        lower_bound = np.clip(average_color - np.array([sensitivity, 50, 50]), 0, 255)
        upper_bound = np.clip(average_color + np.array([sensitivity, 255, 255]), 0, 255)
        
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

        # Transition to next calibration state or end calibration
        if self.calibration_state == 'green':
            self.calibration_state = 'blue'
        elif self.calibration_state == 'blue':
            self.calibration_state = 'yellow'
        elif self.calibration_state == 'yellow':
            self.calibration_state = None  # Calibration done
            print("Calibration done for all colors.")

    # def start_calibration(self, color):
    #     """Method to start the calibration process for a specific color."""
    #     self.calibration_state = color







class HumanTracker:
    def __init__(self):
        self.colors = {
            'yellow': ColorTracker('yellow', [20, 100, 100], [30, 255, 255], {
                'transitionMatrix': [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],
                'measurementMatrix': [[1, 0, 0, 0], [0, 1, 0, 0]],
                'processNoise': 0.03,
                'measurementNoise': 0.001,
                'errorCovPost': 0.1
            }),
            'green': ColorTracker('green', [50, 100, 100], [70, 255, 255], {
                'transitionMatrix': [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],
                'measurementMatrix': [[1, 0, 0, 0], [0, 1, 0, 0]],
                'processNoise': 0.03,
                'measurementNoise': 0.001,
                'errorCovPost': 0.1
            }),
            'blue': ColorTracker('blue', [110, 100, 100], [130, 255, 255], {
                'transitionMatrix': [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],
                'measurementMatrix': [[1, 0, 0, 0], [0, 1, 0, 0]],
                'processNoise': 0.03,
                'measurementNoise': 0.001,
                'errorCovPost': 0.1
            })
        }
        self.calibration_state = 'green'  # 초기 캘리브레이션 상태를 'green'으로 설정

        self.preCalibrateGreen = None
        self.preCalibrateBlue = None
        self.preCalibrateYellow = None

    def preCalibrateColor(self, green, blue, yellow):
        self.preCalibrateGreen = green
        self.preCalibrateBlue = blue
        self.preCalibrateYellow = yellow
        self.colors['green'].preCalibrate(self.preCalibrateGreen)
        self.colors['blue'].preCalibrate(self.preCalibrateBlue)
        self.colors['yellow'].preCalibrate(self.preCalibrateYellow)

        self.calibration_state = None

        


    def process_frame(self, frame):
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        calibration_box = (center_x - 25, center_y - 25, 50, 50)
        if frame is not None:
            if self.calibration_state is not None:
                color_map = {'green': (0, 255, 0), 'blue': (255, 0, 0), 'yellow': (0, 255, 255)}
                cv2.rectangle(frame, (calibration_box[0], calibration_box[1]),
                            (calibration_box[0] + calibration_box[2], calibration_box[1] + calibration_box[3]),
                            color_map[self.calibration_state], 2)
            if cv2.waitKey(1) & 0xFF == ord('k') and self.calibration_state is not None:
                self.calibrate_current_color(frame)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            for color_tracker in self.colors.values():
                color_tracker.update(frame, hsv)
        else:
            print("empty frame")
        return frame

    def calibrate_current_color(self, frame):
        color_tracker = self.colors[self.calibration_state]
        
        color_tracker.calibrate(frame)

        if self.calibration_state == 'green':
            self.calibration_state = 'blue'
        elif self.calibration_state == 'blue':
            self.calibration_state = 'yellow'
        elif self.calibration_state == 'yellow':
            self.calibration_state = None 

    def isCalibrationDone(self):
        return self.calibration_state == None

    def get_color_info(self, color_name):
        color_tracker = self.colors.get(color_name)
        if color_tracker:
            return {
                'position': color_tracker.position,
                'speed'   : color_tracker.speed,
                'velocity': color_tracker.velocity,
                'direction': color_tracker.direction
            }
        else:
            return None 


