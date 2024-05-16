import time
import numpy as np
from collections import deque

class PunchData:
    def __init__(self, max_frames=500):
        self.max_frames = max_frames
        self.speeds = deque(maxlen=self.max_frames)
        self.accelerations = deque(maxlen=self.max_frames)
        self.directions = deque(maxlen=self.max_frames)

    def add_data(self, speed, acceleration, direction):
        self.speeds.append(speed)
        self.accelerations.append(acceleration)
        self.directions.append(direction)

    def get_latest_speed(self):
        return self.speeds[-1] if self.speeds else None

    def get_slice_of_data(self):
        return list(self.speeds)[-500:]

class PunchDataNumpy:
    def __init__(self, max_frames=500):
        self.max_frames = max_frames
        self.speeds = np.zeros(self.max_frames)
        self.accelerations = np.zeros(self.max_frames)
        self.directions = np.zeros(self.max_frames)
        self.last = -1

    def add_data(self, speed, acceleration, direction):
        self.last = (self.last + 1) % self.max_frames
        self.speeds[self.last] = speed
        self.accelerations[self.last] = acceleration
        self.directions[self.last] = direction

    def get_latest_speed(self):
        return self.speeds[self.last] if self.last != -1 else None

    def get_slice_of_data(self):
        start_index = (self.last - 499) % self.max_frames if self.last >= 49 else 0
        end_index = self.last + 1
        if start_index <= end_index:
            return self.speeds[start_index:end_index]
        else:
            return np.concatenate((self.speeds[start_index:], self.speeds[:end_index]))

def measure_performance(class_instance, num_operations):
    start_time = time.time()
    for i in range(num_operations):
        class_instance.add_data(i * 0.1, i * 0.1, i * 0.1)
    end_time = time.time()
    print(f"Time to add {num_operations} elements: {end_time - start_time} seconds")

    start_time = time.time()
    slice_data = class_instance.get_slice_of_data()
    end_time = time.time()
    print(f"Time to retrieve last 50 elements: {end_time - start_time} seconds")
    print(f"Retrieved data sample: {slice_data[:5]}")  # Show first 5 items of the slice for reference

# Test with deque
print("Testing with PunchData (using deque)")
deque_test = PunchData(max_frames=500)
measure_performance(deque_test, 1000)

# Test with numpy
print("Testing with PunchDataNumpy (using numpy arrays)")
numpy_test = PunchDataNumpy(max_frames=500)
measure_performance(numpy_test, 1000)
