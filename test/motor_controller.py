import math
import time

# Mocking the afmotor constants and structure
class MockAFMotor:
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    RELEASE = "RELEASE"
    
    class AF_DCMotor:
        def __init__(self, port):
            self.port = port
            self.speed = 0
            self.direction = None

        def set_speed(self, speed):
            self.speed = speed
            print(f"[Motor {self.port}] Speed set to {speed}")

        def run(self, command):
            self.direction = command
            print(f"[Motor {self.port}] Running: {command}")

# Mocking the kinematic and DTO dependencies
class MockRK:
    speeds = [50, 100, 150, 200, 255]
    def get_deltaT(self, vL, vR, deg): return 0.5
    def calc_fwd_time(self, speed): return 1.0
    def get_robot_turn(self, vl, vr, dt): return (90, 10)
    def calcRobotPos(self, x, y, th, dO, irc): return (5, 5)

class MockGridDto:
    def __init__(self):
        class Lock:
            def acquire(self): pass
            def release(self): pass
        self._lock_dist = Lock()
    def get_unit(self): return 10



## MockMotorController Implementation

class MockMotorController:
    motor_start_time = 0.1  # Reduced for faster testing

    def __init__(self, vMode, dto=None):
        self.dto = dto or MockGridDto()
        self.unit = self.dto.get_unit()
        self.vMode = vMode
        self.totalDistance = 0
        self.totalTime = 0
        self.speed_steps = (50, 100, 150, 200, 255)
        
        print("Initializing Mock Motors...")
        self.motors = {i: MockAFMotor.AF_DCMotor(i) for i in range(1, 5)}
        
        self.speed_step = self.speed_steps[vMode] if vMode < len(self.speed_steps) else 50
        self._set_speed(self.speed_step)

    def _set_speed(self, speed):
        for m in self.motors.values():
            m.set_speed(speed)

    def forward(self):
        print("\n--- Moving Forward ---")
        self.dto._lock_dist.acquire()
        for m in self.motors.values():
            m.run(MockAFMotor.FORWARD)
        
        # In a real mock, you might skip time.sleep to run tests faster
        duration = 1.0 + self.motor_start_time
        self.totalTime += duration
        self.totalDistance += self.unit
        self.dto._lock_dist.release()
        print(f"Moved {self.unit} units. Total Dist: {self.totalDistance}")

    def get_speed(self, ix: int):
        return 150  # Fixed speed for testing
    def get_turn_deltaT(self, vL, vR, deg):
        return 1
    def calc_fwd_time(self):
        return 2
    

    def copmute_arc_distance(self,vl,vr,deltaT):
        return 150
    
    def stop(self):
        print("\n--- Stopping ---")
        for m in self.motors.values():
            m.run(MockAFMotor.RELEASE)