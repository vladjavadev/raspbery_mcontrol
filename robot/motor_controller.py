from afmotor import afmotor as af
from data.grid_dto import GridDto
import robot.kinematic as rk
import time
import math


class MotorController:

    speed_steps=(50,100,150,200,255)

    def __get_speed_step(self,vMode):
        if vMode<len(self.speed_steps)-1 and vMode>=0:
            return self.speed_steps[vMode]
        else:
            return self.speed_steps[0]
        
    def __init_motors(self):
        self.motor4 = af.AF_DCMotor(4)
        self.motor3 = af.AF_DCMotor(3)
        self.motor2 = af.AF_DCMotor(2)
        self.motor1 = af.AF_DCMotor(1)

    def __set_speed(self, speed_step):
        self.motor4.set_speed(speed_step)
        self.motor3.set_speed(speed_step)
        self.motor2.set_speed(speed_step)
        self.motor1.set_speed(speed_step)

    def __init__(self, vMode, dto: GridDto):
        self.totalDistance = 0
        self.totalTime = 0
        self.dto=dto
        self.unit = dto.get_unit()
        self.vMode = vMode
        self.__init_motors()
        self.speed_step = self.__get_speed_step(vMode)
        self.__set_speed(self.speed_step)




    def get_speed(self, ix: int):
        return rk.speeds[ix-1]
    
    def get_turn_deltaT(self, vL, vR, deg):
        return rk.get_deltaT(vL, vR, deg)
    def calc_fwd_time(self):
        return self.unit/rk.speeds[self.vMode]
    

    def copmute_arc_distance(self,vl,vr,deltaT):
        dOmega, irc = rk.get_robot_turn(vl, vr, deltaT) 
        x1, y1 =rk.calcRobotPos(0,0,0,dOmega,irc)
        dist=math.sqrt(x1**2 + y1**2)
        return dist
    
    def forward(self):
        self.dto._lock_dist.acquire()

        self.motor4.run(af.FORWARD)
        self.motor3.run(af.FORWARD) 
        self.motor2.run(af.FORWARD)
        self.motor1.run(af.FORWARD)

        timeSleep = self.calc_fwd_time()
        time.sleep(timeSleep)

        self.totalTime+=timeSleep
        self.totalDistance+=self.unit
        self.dto._lock_dist.release()

        
    def reverse(self):
        self.dto._lock_dist.acquire()

        timeSleep = self.calc_fwd_time(self.speed_step)

        self.motor4.run(af.BACKWARD)
        self.motor3.run(af.BACKWARD) 
        self.motor2.run(af.BACKWARD)
        self.motor1.run(af.BACKWARD)

        time.sleep(timeSleep)
        self.totalTime+=timeSleep
        self.totalDistance+=self.unit
        self.dto._lock_dist.release()


    def turn_right(self, step):
        self.dto._lock_dist.acquire()

        vL = rk.speeds[self.vMode]
        vR = rk.speeds[self.vMode]
        deltaT = rk.get_deltaT(0, vR, step)

        self.motor4.run(af.FORWARD)
        self.motor3.run(af.FORWARD) 
        self.motor2.run(af.BACKWARD)
        self.motor1.run(af.BACKWARD)
        time.sleep(deltaT)
        self.totalTime+=deltaT
        self.totalDistance+=self.unit
        self.dto._lock_dist.release()


    def turn_left(self, step):
        self.dto._lock_dist.acquire()

        vL = rk.speeds[self.vMode]
        vR = rk.speeds[self.vMode]
        deltaT = rk.get_deltaT(vL, 0, step)

        self.motor4.run(af.BACKWARD)
        self.motor3.run(af.BACKWARD) 
        self.motor2.run(af.FORWARD)
        self.motor1.run(af.FORWARD)
        time.sleep(deltaT)
        self.totalTime+=deltaT
        self.totalDistance+=self.unit
        self.dto._lock_dist.release()

    def stop(self):
        self.motor4.run(af.RELEASE)
        self.motor3.run(af.RELEASE) 
        self.motor2.run(af.RELEASE)
        self.motor1.run(af.RELEASE)