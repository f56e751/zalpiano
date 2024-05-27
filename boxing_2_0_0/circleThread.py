import threading
import time
import math

class CircularActionGenerator:
    def __init__(self, radius, num_points, interval):
        self.radius = radius
        self.num_points = num_points
        self.interval = interval
        self.points = self.generate_circle_points(self.radius, self.num_points)
        self.current_index = 0
        self.stop_event = threading.Event()

    def generate_circle_points(self, radius, num_points):
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append((x, y))
        return points

    def circularAction(self):
        if self.current_index >= self.num_points:
            self.current_index = 0
        point = self.points[self.current_index]
        self.current_index += 1
        return point

    def start(self):
        self.stop_event.clear()
        self.schedule_next_action()

    def schedule_next_action(self):
        if not self.stop_event.is_set():
            point = self.circularAction()
            print(f"Current Point: {point}")
            threading.Timer(self.interval, self.schedule_next_action).start()

    def stop(self):
        self.stop_event.set()

# 사용 예시
if __name__ == '__main__':
    generator = CircularActionGenerator(radius=10, num_points=20, interval=1)
    generator.start()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        generator.stop()
        print("Stopped by user")
