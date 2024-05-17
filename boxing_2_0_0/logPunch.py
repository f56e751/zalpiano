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
    
    def get_data_last_seconds(self, last_seconds):
        current_time = time.time()
        index = len(self.times) - 1
        # Find the oldest time within the last n seconds
        while index >= 0 and (current_time - self.times[index]) <= last_seconds:
            index -= 1
        index += 1  # adjust index to include the first relevant data point
        
        # Return slices of the deques from the calculated index to the end
        return {
            'velocities': list(self.vels)[index:],
            'speeds': list(self.speeds)[index:],
            'accelerations': list(self.accelerations)[index:],
            'directions': list(self.directions)[index:],
            'times': list(self.times)[index:]
        }


    def showSpeeds(self):
        print(self.speeds)

    def get_slice_of_data(self):
        return list(self.speeds)[-500:]


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

    




import numpy as np
import time

class PunchDataNumpy:
    def __init__(self, max_frames=500):
        self.max_frames = max_frames
        self.speeds = np.zeros(self.max_frames)
        self.vels = np.zeros((self.max_frames, 2))
        self.accelerations = np.zeros(self.max_frames)
        self.accDirections = np.zeros(self.max_frames)
        self.directions = np.zeros(self.max_frames)
        self.times = np.zeros(self.max_frames)  # Array to store timestamps
        self.last = -1  # Starts at -1 because 0 will be the index of the first element

    def add_data(self, vel, speed, direction, time):
        self.last = (self.last + 1) % self.max_frames 
        self.vels[self.last] = vel
        self.speeds[self.last] = speed
        # self.accelerations[self.last] = acceleration
        self.directions[self.last] = direction
        self.times[self.last] = time # Store current timestamp
        self.calculate_current_acceleration()

    # def add_data(self, speed, acceleration, direction):
    #     self.last = (self.last + 1) % self.max_frames
    #     self.speeds[self.last] = speed
    #     self.directions[self.last] = direction
    #     vel_x = speed * np.cos(np.radians(direction))
    #     vel_y = speed * np.sin(np.radians(direction))
    #     self.vels[self.last] = [vel_x, vel_y]
    #     self.times[self.last] = time.time()
    #     self.calculate_current_acceleration()

    def calculate_current_acceleration(self):
        if self.last > 0:
            prev_index = (self.last - 1) % self.max_frames
            current_acceleration = Acceleration()
            current_acceleration.calculateAcc(self.directions[prev_index], self.speeds[prev_index], self.vels[prev_index],
                                              self.directions[self.last], self.speeds[self.last], self.vels[self.last])
            self.accelerations[self.last] = current_acceleration.getMagnitude()
            self.accDirections[self.last] = current_acceleration.getDirection()

    def get_latest_speed(self):
        return self.speeds[self.last] if self.last != -1 else None

    def get_latest_acceleration(self):
        return self.accelerations[self.last] if self.last != -1 else None

    def get_latest_direction(self):
        return self.directions[self.last] if self.last != -1 else None

    def get_data_last_seconds(self, last_seconds):
        # current_time = time.time()
        current_time = self.times[self.last]
        # Create a mask to filter data within the last n seconds
        valid_indices = (current_time - self.times) <= last_seconds
        return {
            'velocities': self.speeds[valid_indices],
            'speeds': self.speeds[valid_indices],
            'accelerations': self.accelerations[valid_indices],
            'directions': self.directions[valid_indices],
            'times': self.times[valid_indices]
        }

    def get_data_last_seconds(self, last_seconds):
        # Use the timestamp at self.last as the reference time
        reference_time = self.times[self.last]
        valid_indices = (reference_time - self.times) <= last_seconds
        return {
            'velocities': self.vels[valid_indices],
            'speeds': self.speeds[valid_indices],
            'accelerations': self.accelerations[valid_indices],
            'directions': self.directions[valid_indices],
            'times': self.times[valid_indices]
        }

    def get_all_speeds(self):
        return np.roll(self.speeds, -self.last-1)

    def get_all_accelerations(self):
        return np.roll(self.accelerations, -self.last-1)

    def get_all_directions(self):
        return np.roll(self.directions, -self.last-1)
    
    def get_all_time(self):
        return np.roll(self.times, -self.last-1)
    
    def get_slice_of_data(self):
        start_index = (self.last - 499) % self.max_frames if self.last >= 49 else 0
        end_index = self.last + 1
        if start_index <= end_index:
            return self.speeds[start_index:end_index]
        else:
            return np.concatenate((self.speeds[start_index:], self.speeds[:end_index]))
