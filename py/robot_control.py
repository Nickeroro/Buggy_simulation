import numpy as np
import time
import random
import sonar_filter as flt


class RobotControl:
    def __init__(self):
        self.distBetweenWheels = 0.12
        self.nTicksPerRevol = 720
        self.wheelDiameter = 0.06

    def testMove(self, rb, speedLeft, speedRight, duration):
        # forward motion
        rb.set_speed(speedLeft, speedRight)
        loopIterTime = 0.050
        tStart = time.time()
        while time.time() - tStart < duration:
            time.sleep(loopIterTime)  # wait
        # stop the robot 
        rb.stop()

    def testInfiniteObstacle(self, rb):
        loopIterTime = 0.050  # duration of loop iteration 50 ms
        legTimeMax = 5.0  # always turn after 5s (even if no obsctacle)
        distObstacle = 0.3  # stops and change direction if obstacle
        # at less than 30 cm
        while True:  # infinite loop : stop by typing ctrl-C
            rb.set_speed(90, 90)
            tStartLeg = time.time()
            while True:
                t0 = time.time()
                if time.time() - tStartLeg >= legTimeMax:
                    break
                distFront = rb.get_sonar("front")
                # print ("distFront :",distFront)
                if distFront != 0.0 and distFront < distObstacle:
                    break
                t1 = time.time()
                dt = loopIterTime - (t1 - t0)
                if dt > 0:
                    time.sleep(dt)  # wait for end of iteration
            # in the case the robot is trapped in front of a wall
            # go back at speed 40 for 0.5 seconds 
            rb.set_speed(-40, -40)
            time.sleep(0.5)
            # set random orientation by setting rotation duration
            # minimum time is 0.2 seconds, max is 1.5 seconds
            rotationTime = 0.2 + 1.3 * random.random()
            rotationDirection = random.random()
            if rotationDirection < 0.5:
                self.testMove(rb, 40, -40, rotationTime)
            else:
                self.testMove(rb, -40, 40, rotationTime)

    def inPlaceTurnRight(self, rb, ang):  # ang in degrees
        odoLeft, odoRight = rb.get_odometers()
        angleLim = 4 * ang
        while True:
            t0 = time.time()
            odoLeftTmp, odoRightTmp = rb.get_odometers()
            error = 4
            if abs((odoLeftTmp - odoLeft) - angleLim) <= error:
                rb.set_speed(0, 0)
                break
            t1 = time.time()
            time.sleep(abs(-(t1 - t0) + 0.05))
            k = 0.01  # un coef pour la vitesse
            err = abs((odoLeftTmp - odoLeft) - angleLim)
            vit = 10 * err * k
            rb.set_speed(vit, -vit)
        # replace with your code

    def inPlaceTurnLeft(self, rb, ang):  # ang in degrees
        odoLeft, odoRight = rb.get_odometers()
        angleLim = 4 * ang
        while True:
            t0 = time.time()
            odoLeftTmp, odoRightTmp = rb.get_odometers()
            error = 4
            if abs((odoRightTmp - odoRight) - angleLim) <= error:
                rb.set_speed(0, 0)
                break
            t1 = time.time()
            time.sleep(abs(-(t1 - t0) + 0.05))
            k = 0.01  # coef qui reglemente la vitesse
            err = abs((odoRightTmp - odoRight) - angleLim)
            vit = 10 * err * k
            rb.set_speed(-vit, vit)
            # replace with your code

    def goLineOdometer(self, rb, dist, speed):
        perim_roue = 0.1885
        dist = dist / perim_roue
        impulsions = dist * 1440
        offsetRight, offsetLeft = rb.get_odometers()
        x = 0
        t0 = 0
        rb.set_speed(speed, speed)
        while x < impulsions:
            t0 = time.time()
            odoRight, odoLeft = rb.get_odometers()
            x = odoLeft - offsetLeft
            t1 = time.time()
            loopIterTime = 0.050
            dt = loopIterTime - (t1 - t0)
            time.sleep(dt)
        rb.stop()

    def followWallsLeft(self, rb, setPoint=0.5, nominalSpeed=50):
        kp = 10
        kd = 1000
        lastError = 0
        deriveOk = False
        filter_sonar = flt.SonarFilter()
        while True:
            distFront = rb.get_sonar('front')
            distFront = filter_sonar.median_filter(distFront)

            print(distFront)
            distWall = rb.get_sonar('left')
            distWall = filter_sonar.median_filter(distWall)

            controlError = setPoint - distWall
            if deriveOk:
                deriveError = controlError - lastError
                deltaSpeed = kp * controlError + kd * deriveError
            else:
                deltaSpeed = kp * controlError

            deltaSpeedMax = 20
            if deltaSpeed > deltaSpeedMax:
                deltaSpeed = deltaSpeedMax
            elif deltaSpeed < -deltaSpeedMax:
                deltaSpeed = -deltaSpeedMax

            rb.set_speed(nominalSpeed + deltaSpeed, nominalSpeed - deltaSpeed)
            lastError = controlError
            deriveOk = True

            if distFront > 0.2 and distFront < 0.44:
                rb.stop()
                break


    def followWallsRight(self, rb, setPoint=0.5, nominalSpeed=50):
        kp = 10
        kd = 1000
        lastError = 0
        deriveOk = False
        filter_sonar = flt.SonarFilter()
        while True:
            distFront = rb.get_sonar('front')
            distFront = filter_sonar.median_filter(distFront)

            distWall = rb.get_sonar('right')
            distWall = filter_sonar.median_filter(distWall)

            print(distWall)

            controlError = setPoint - distWall
            if deriveOk:
                deriveError = controlError - lastError
                deltaSpeed = kp * controlError + kd * deriveError
            else:
                deltaSpeed = kp * controlError

            deltaSpeedMax = 20
            if deltaSpeed > deltaSpeedMax:
                deltaSpeed = deltaSpeedMax
            elif deltaSpeed < -deltaSpeedMax:
                deltaSpeed = -deltaSpeedMax

            rb.set_speed(nominalSpeed - deltaSpeed, nominalSpeed + deltaSpeed)
            lastError = controlError
            deriveOk = True

            if distWall == 0:
                rb.stop()
                break