from collections import deque
import numpy as np
import time

class Acceleration():
    def __init__(self):
        self.direction = 0
        self.magnitude = 0

    def calculateAcc(self, preDirection, preSpeed, preVel, currDirection, currSpeed, currVel):
        angleDiff = abs(currDirection - preDirection)
        if angleDiff > 180:
            angleDiff = 360 - angleDiff

        # self.direction = 
        self.magnitude = np.sqrt(np.power(currSpeed,2) + np.power(preSpeed,2) - 2 * currSpeed * preSpeed * np.cos(np.deg2rad(angleDiff)))
        self.direction = np.degrees(np.arctan2(currVel[1] - preVel[1], currVel[0] - preVel[0]))

    def getMagnitude(self):
        return self.magnitude
    
    def getDirection(self):
        return self.direction
        
        
class PunchData():
    def __init__(self, max_frames=500):
        self.max_frames = max_frames
        self.vels = deque(maxlen=self.max_frames)
        self.speeds = deque(maxlen=self.max_frames)
        self.accelerations = deque(maxlen=self.max_frames)
        self.directions = deque(maxlen=self.max_frames)
        self.times = deque(maxlen=self.max_frames)

    def add_data(self, vel, speed, direction, time):
        self.vels.append(vel)
        self.speeds.append(speed)
        self.directions.append(direction)
        self.times.append(time)
        self.accelerations.append(self.getCurrentAcc())

    def getCurrentAcc(self):
        acceleration = Acceleration()
        if len(self.speeds) > 0:
            lastIndex = len(self.speeds) - 1
            acceleration.calculateAcc(self.directions[lastIndex - 1], self.speeds[lastIndex - 1], self.vels[lastIndex - 1],
                                      self.directions[lastIndex], self.speeds[lastIndex], self.vels[lastIndex])
        return acceleration

    def get_latest_speed(self):
        return self.speeds[-1] if self.speeds else None

    def get_latest_acceleration(self):
        return self.accelerations[-1] if self.accelerations else None

    def get_latest_direction(self):
        return self.directions[-1] if self.directions else None
    
    def showSpeeds(self):
        print(self.speeds)

    # def 

if __name__ == "__main__":
    PunchData = PunchData()
    num_operations = 10
    for i in range(num_operations):
        PunchData.add_data(i, i, i)
        # print(f"size is {len(PunchData.speeds)}")
        # print(f"1 index is: {PunchData.speeds[0]}")
        # print(PunchData.speeds)


    def test_acceleration():
        test_cases = [
            (0, 0, [0, 0], 0, 10, [10, 0]),  # Acceleration from 0 to 10 m/s in the east direction
            (0, 10, [10, 0], 90, 10, [0, 10]),  # Change direction from east to north with constant speed
            (45, 5, [np.sqrt(2), np.sqrt(2)], 135, 10, [0, 10]),  # Accelerating and turning
        ]

        for i, (preDir, preSpeed, preVel, curDir, curSpeed, curVel) in enumerate(test_cases):
            acc = Acceleration()
            magnitude, direction = acc.calculateAcc(preDir, preSpeed, preVel, curDir, curSpeed, curVel)
            print(f"Test Case {i+1}:")
            print(f"  Previous Direction: {preDir}°, Speed: {preSpeed} m/s")
            print(f"  Current Direction: {curDir}°, Speed: {curSpeed} m/s")
            print(f"  Calculated Acceleration Magnitude: {magnitude:.2f} m/s²")
            print(f"  Calculated Acceleration Direction: {direction:.2f}°\n")

    # Call the test function
    test_acceleration()


class PunchDataNumpy:
    # Slower than deque based Class
    def __init__(self, max_frames=500):
        self.max_frames = max_frames
        self.speeds = np.zeros(self.max_frames)
        self.accelerations = np.zeros(self.max_frames)
        self.directions = np.zeros(self.max_frames)
        self.last = -1  # Starts at -1 because 0 will be the index of the first element

    def add_data(self, speed, acceleration, direction):
        self.last = (self.last + 1) % self.max_frames  # Increment and wrap around
        self.speeds[self.last] = speed
        self.accelerations[self.last] = acceleration
        self.directions[self.last] = direction

    def get_latest_speed(self):
        return self.speeds[self.last] if self.last != -1 else None

    def get_latest_acceleration(self):
        return self.accelerations[self.last] if self.last != -1 else None

    def get_latest_direction(self):
        return self.directions[self.last] if self.last != -1 else None

    def get_all_speeds(self):
        # Return the array with the oldest data first
        return np.roll(self.speeds, -self.last-1)

    def get_all_accelerations(self):
        # Return the array with the oldest data first
        return np.roll(self.accelerations, -self.last-1)

    def get_all_directions(self):
        # Return the array with the oldest data first
        return np.roll(self.directions, -self.last-1)
