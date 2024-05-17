import numpy as np
import time
from timeit import timeit
from logPunch import PunchData, PunchDataNumpy


def populate_data(punch_data_class, n=100000, max_frames=50000):
    # Populate the data structure with a large number of data points
    start_time = time.time() - n * 0.01  # Start from 0.01 seconds ago per data point
    current_time = start_time
    for i in range(n):
        vel = np.array([np.random.rand(), np.random.rand()])
        speed = np.random.rand() * 10
        direction = np.random.rand() * 360
        timestamp = time.time() - np.random.rand() * 100  # Random time within the last 100 seconds
        current_time += 0.01
        punch_data_class.add_data(vel, speed, direction, current_time)
    # return punch_data_batch

def benchmark_get_data_last_seconds(punch_data_class, last_seconds=10):
    # Function to be timed
    def run_test():
        result = punch_data_class.get_data_last_seconds(last_seconds)
        # Uncomment to print the lengths of the retrieved data arrays
        # print({key: len(value) for key, value in result.items()})
    
    # Run the benchmark
    duration = timeit(run_test, number=100)
    return duration

def benchmark_add_data(punch_data_class, n=100000):
    # Start time calculation moved inside the benchmark function for proper scoping
    
    def populate():
        # Declare nonlocal if you need to modify start_time within this function
        current_time = 1715922911.28
        for i in range(n):
            vel = np.array([np.random.rand(), np.random.rand()])
            speed = np.random.rand() * 10
            direction = np.random.rand() * 360
            # Use current_time instead of recomputing from time.time()
            current_time += 0.01
            punch_data_class.add_data(vel, speed, direction, current_time)

    # Run the benchmark using timeit, which correctly handles the timing of populate()
    duration = timeit(populate, number=1)
    return duration





def measure_performance(class_instance, num_operations):
    print(f"{class_instance} test")
    start_time = time.time()
    benchmark_add_data(class_instance, num_operations)
    end_time = time.time()
    print(f"Time to add {num_operations} elements: {end_time - start_time} seconds, avg: {(end_time - start_time) / float(num_operations)}")

    start_time = time.time()
    slice_data = class_instance.get_slice_of_data()
    end_time = time.time()
    print(f"Time to retrieve last 50 elements: {end_time - start_time} seconds")
    # print(f"Retrieved data sample: {slice_data}")  # Show first 5 items of the slice for reference

    sliceTime = 0.2
    start_time = time.time()
    benchmark_get_data_last_seconds(class_instance, sliceTime)
    end_time = time.time()    
    print(f"Time to silce {sliceTime}sec elements: {end_time - start_time} seconds")



# Create instances of both classes
deque_punch_data = PunchData(max_frames=500)
numpy_punch_data = PunchDataNumpy(max_frames=500)

# Benchmarking addition of data
n = 100000


measure_performance(deque_punch_data, n)
measure_performance(numpy_punch_data, n)

# deque_add_duration = benchmark_add_data(deque_punch_data, n)
# numpy_add_duration = benchmark_add_data(numpy_punch_data, n)

# print(f"Adding data to deque-based class took {deque_add_duration:.5f} seconds, avg: {deque_add_duration / n:}")
# print(f"Adding data to NumPy-based class took {numpy_add_duration:.5f} seconds, avg: {numpy_add_duration / n:}")

# # Benchmarking get_data_last_seconds
# last_seconds = 0.2
# deque_duration = benchmark_get_data_last_seconds(deque_punch_data, last_seconds)
# numpy_duration = benchmark_get_data_last_seconds(numpy_punch_data, last_seconds)
# # np.set_printoptions(threshold=np.inf)e
# # np.set_printoptions(formatter={'all': lambda x: f'{int(x):d}'})
# np.set_printoptions(formatter={'float': '{:0.10f}'.format})
# print(numpy_punch_data.get_all_time())
# print(type(numpy_punch_data.get_all_time()))

# print(f"Retrieving last {last_seconds} seconds of data from deque-based class took {deque_duration:.5f} seconds")
# print(f"Retrieving last {last_seconds} seconds of data from NumPy-based class took {numpy_duration:.5f} seconds")

