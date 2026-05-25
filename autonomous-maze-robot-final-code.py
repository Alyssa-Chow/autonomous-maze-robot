#final project 3 code
#-----------------------------------------------------------------------------------------------------------------
# INITIALIZE AND IMPORTS
#-----------------------------------------------------------------------------------------------------------------
#imports
from buildhat import Motor
from basehat import UltrasonicSensor
from basehat import IRSensor
from basehat import IMUSensor
import numpy as np
import math
import time

#initialize drive motors
motorR = Motor('A')
motorL = Motor('B')
#initialize flatbed motor...check port
motorC = Motor('D')


# ultrasonic sensors
ultraRtPin = 22
ultraLtPin = 24
ultraFtPin = 26
ultraRt = UltrasonicSensor(ultraRtPin)
ultraLt = UltrasonicSensor(ultraLtPin)
ultraFt = UltrasonicSensor(ultraFtPin)

#initialize IMU sensor
IMU = IMUSensor()

#initialize IR sensor
infraPin1 = 0
infraPin2 = infraPin1 + 1
IR = IRSensor(infraPin1, infraPin2)
#------------------------------------------------------------------------------------------------------------------
#UDFS
#------------------------------------------------------------------------------------------------------------------
def driveStraight(speedInput):
    print("driveStraight")
                 
    desiredDist = 40
    wheelDiam = 7.833
    desiredPosR = desiredDist / (3.14 * wheelDiam)
    desiredPosL = -desiredDist / (3.14 * wheelDiam)
    
    motorR.run_for_rotations(-desiredPosR,-speedInput,False) 
    motorL.run_for_rotations(-desiredPosL,-speedInput,True)
    
def turnRight(speedInput):
    print("turnRight")
    motorR.run_for_rotations(.5,-speedInput,False) 
    motorL.run_for_rotations(.5,-speedInput,True)
    
def turnLeft(speedInput):
    print("turnLeft")
    motorR.run_for_rotations(.44,speedInput,False) 
    motorL.run_for_rotations(.44,speedInput,True)

def openCargo(speedInput):
    motorC.run_for_rotations(-0.25,-speedInput,True)
    
def closeCargo(speedInput):
    motorC.run_for_rotations(-0.25,speedInput,True)
    
def stopDriving():
    print("stopDriving")
    motorL.stop()
    motorR.stop()
    motorL.float()
    motorR.float()
#-----------------------------------------------------------------------------------------------------------------
#MAIN
#-----------------------------------------------------------------------------------------------------------------
def main():
    try:
        stopDriving()
        while True:
            try:
                #--------------------------------------------------------------------------------------------------
                #ADJUST RELEVANT VALS
                #--------------------------------------------------------------------------------------------------
                #driving straight
                speedInput = 10
                tStep = 0.001

                # mapping PATH and HAZARDS
                mapRows = 4
                mapCols = 5
                mapp = np.zeros((mapRows, mapCols), dtype = int)
                mapColPrev = 0
                mapRowPrev = 0
                face = 1
                hazards = np.zeros((mapRows, mapCols), dtype = int)
                haz2X = 0
                haz2Y = 0
                haz3X = 0
                haz3Y = 0

                # wall distance for R&L ultrasonic sensors
                wallDist = (40 - 14.2) / 2 #where 14.2 is the distance from R-->L ultrasonic sensors on bot

                #travel distance, which is equal to one unit in cm of the grid
                unitDist = 40

                # front ultrasonic distance the robot needs to be from thewall in order to make clear turns
                turnDist = 15

                # coordinate count, which is teh number of cells traveled across at that time. NOTE: this is NOT the total number of coords added to map that inclued hazards. this is necessary to mod the motor position thing for determining travel distance
                coordCt = 1
                
                #desired radius to stay away from IR and IMUs, CHANGE AS NEEDED
                desiredIRval = 40
                desiredIMUval = 40
                
                #-------------------------------------------------------------------------------------------------
                
                #get infra vals
                value1 = IR.value1
                value2 = IR.value2 #this value is kinda weird
                
                #equation for radius/distance the robot is away from the IR sensor
                IRradius = (value1 - 317) / (-3.95)
                          
                # Reading magnet values and printing them
                x, y, z = IMU.getMag()
                if x > 0 or y > 0 or z > 0:     
                    IMUmagn = math.sqrt(x**2 + y**2 + z**2)
                    IMUradius = IMUmagn**(-1/1.65) * 0.004149637
                else:
                    IMUradius = 10000
                    IMUmagn = 0
                
            
                #get the current dist val from rt,lt,and ft ultra sensors and fix 'nones'. wait to take reading for 1 sec
                ultValRt = ultraRt.getDist
                if ultValRt == None:
                    ultValRt = 100  
                ultValLt = ultraLt.getDist
                if ultValLt == None:
                    ultValLt = 100
                ultValFt = ultraFt.getDist
                if ultValFt == None:
                    ultValFt = 100
                print('FIRST ultValRt:{} cm'.format(ultValRt))
                print('FIRST ultValLt:{} cm'.format(ultValLt))
                print('FIRST ultValFt:{} cm'.format(ultValFt))
                
                #add new travel to map and increase coordinate count (number of coordinates traveled to NOT number of total coords on map including hazards)
                if coordCt == 1: #put a 5 on the first coord
                    driveStraight(speedInput)
                    mapp[mapRowPrev, mapColPrev] = 5
                    origRow = mapRowPrev
                    origCol = mapColPrev
                elif (face % 2) == 0: #even face control cols
                    if ultValFt >= 100: #change 100 if needed?????
                        mapp[mapRowPrev, mapColPrev + 1] = 4 #add a 4 to the last coordinate
                    elif face == 2: 
                        mapp[mapRowPrev, mapColPrev + 1] = 1
                    elif face == 4:
                        mapp[mapRowPrev, mapColPrev - 1] = 1
                    coordCt += 1 #increase coordinate count described above. this is necessary to mod the motor position thing for determining travel distance
                elif (face % 2) != 0: #odd face control rows
                    if ultValFt >= 100:
                        mapp[mapRowPrev + 1, mapColPrev] = 4 #add a 4 to the last coordinate
                    elif face == 1:
                        mapp[mapRowPrev + 1, mapColPrev] = 1
                    elif face == 3:
                        mapp[mapRowPrev - 1, mapColPrev] = 1
                    coordCt += 1
                print("CoordCt: ", coordCt)
                print("Face: ", face)
                
                
                #update face...
                #if the distance from the wall to ultra sensor is greater 40, then turn left or right in that direciton and update face accordingly
                if ultValRt > 40: 
                    turnRight(speedInput)
                    face += 1
                    if face == 5:
                        face = 1
                    if IRradius > desiredIRval and IMUradius > desiredIMUval: 
                        driveStraight(speedInput)
                    else:
                        if IRradius < desiredIRval or IMUradius < desiredIMUval:
                            if IRradius < desiredIRval:
                                key = 2
                            if IMUradius < desiredIMUval:
                                key = 3
                            if (face % 2) == 0: #even face control cols
                                if face == 2: 
                                    hazards[mapRowPrev, mapColPrev + 1] = key
                                elif face == 4:
                                    hazards[mapRowPrev, mapColPrev - 1] = key
                            elif (face % 2) != 0: #odd face control rows
                                if face == 1:
                                    hazards[mapRowPrev + 1, mapColPrev] = key
                                elif face == 3:
                                    mapp[mapRowPrev - 1, mapColPrev] = key
                            turnRight(speedInput)
                            face += 1
                            if face == 5:
                                face = 1
                elif ultValFt > 40 and IRradius < desiredIRval and IMUradius < desiredIMUval:
                    driveStraight(speedInput)
                elif ultValLt > 40: # dec. face for CCW rotation
                    if IRradius < desiredIRval:
                        key = 2
                    if IMUradius < desiredIMUval:
                        key = 3
                    if (face % 2) == 0: #even face control cols
                        if face == 2: 
                            hazards[mapRowPrev, mapColPrev + 1] = key
                            if key == 2:
                                haz2X = mapColPrev + 1
                                haz2Y = mapRowPrev
                            elif key == 3:
                                haz3X = mapColPrev + 1
                                haz3Y = mapRowPrev
                        elif face == 4:
                            hazards[mapRowPrev, mapColPrev - 1] = key
                            if key == 2:
                                haz2X = mapColPrev - 1
                                haz2Y = mapRowPrev
                            elif key == 3:
                                haz3X = mapColPrev - 1
                                haz3Y = mapRowPrev
                        elif (face % 2) != 0: #odd face control rows
                            if face == 1:
                                hazards[mapRowPrev + 1, mapColPrev] = key
                                if key == 2:
                                    haz2X = mapColPrev
                                    haz2Y = mapRowPrev + 1
                                elif key == 3:
                                    haz3X = mapColPrev
                                    haz3Y = mapRowPrev + 1
                            elif face == 3:
                                hazards[mapRowPrev - 1, mapColPrev] = key
                                if key == 2:
                                    haz2X = mapColPrev
                                    haz2Y = mapRowPrev - 1
                                elif key == 3:
                                    haz3X = mapColPrev
                                    haz3Y = mapRowPrev - 1
                    turnLeft(speedInput)
                    face -= 1
                    if face == 0:
                        face = 4
                    if IRradius > desiredIRval and IMUradius > desiredIMUval: 
                        driveStraight(speedInput)
                    else:
                        if IRradius < desiredIRval or IMUradius < desiredIMUval:
                            turnLeft(speedInput)
                            face -= 1
                            if face == 0:
                                face = 4
                else:
                    turnRight(speedInput)
                    face += 1
                    if face == 5:
                        face = 1
                
                
                #IF YOU ARE OUT OF THE MAZE.......
                if ultValFt >= 80 and ultValRt >= 80 and ultValLt >= 80: #ADJUST THIS AT DEMO
                    if ultValFt >= 100: #change 100 if needed????? #added these down here from mapping for path
                        mapp[mapRowPrev, mapColPrev + 1] = 4
                    with open("team82_map.csv", "w") as f:
                        f.write("Team: 82\n")
                        f.write("Map: 1 \n")
                        f.write("Unit: cm \n")
                        f.write(f"Origin: ({origCol}, {origRow})\n")
                        f.write("Notes: \n")
                        for i in range (mapRows):
                            for j in range (mapCols):
                                f.write(str(mapp[i,j]))
                            f.write("\n")
                        #this file is closed
                    stopDriving()
                    with open ("team82_hazards.csv", "w") as f:
                        f.write("Team: 82\n")
                        f.write("Map: 1\n")
                        f.write("Notes: This is a description of the hazards.\n")
                        f.write("\n")
                        f.write("Hazard Type, Paramater of Interest, Parameter Value, Hazard X Coordinate (cm), Hazard Y Coordinate (cm)\n")
                        f.write(f"Electrical/Magnetic Activity Source, Field Strength (uT), {IMUmagn}, {10 * haz3X}, {10 * haz3Y}\n")
                        f.write(f"High Temperature Heat Source, Radiated Power (W), {value1}, {10 * haz2X}, {10 * haz2Y}\n")
                        #this file is closed
                    stopDriving()
                    openCargo(speedInput)
                    time.sleep(3)
                    closeCargo(speedInput)
                        
                
                #time sleep for entire loop      
                time.sleep(0.1)
        
            except IOError:
                print("\nError occured while attempting to read values.")
                break
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting...")
        stopDriving()
        time.sleep(1)
if __name__ == '__main__':
    main()