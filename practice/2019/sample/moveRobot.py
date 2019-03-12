import vrep
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

class Robot:
    def __init__(self):
        vrep.simxFinish(-1)
        self.client_id = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
        assert self.client_id != -1, 'Failed connecting to remote API server'
        e = vrep.simxStartSimulation(self.client_id, vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "Unable to start simulation", True)
        e, self.left_motor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor',
                                                        vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "Can not find left motor", True)
        e, self.right_motor_handle = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor',
                                                         vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "Can not find right motor", True)
        e, self.proximity_handle = vrep.simxGetObjectHandle(self.client_id, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "Can not find proximity sensor", True)
        e, self.lidar_handle1 = vrep.simxGetObjectHandle(self.client_id, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "Can not find lidar 1", True)
        e, self.lidar_handle2 = vrep.simxGetObjectHandle(self.client_id, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "Can not find lidar 2", True)
        sec, msec = vrep.simxGetPingTime(self.client_id)
        print("Ping time: %f" % (sec + msec / 1000.0))
        print("Initialization finished")

    def check_error_code(self, error_code, message, need_exit = False):
        if error_code != vrep.simx_return_ok:
            print("ERROR: Code {}. {}".format(error_code, message))
            if need_exit:
                sys.exit()

    def set_motor_speed(self, left_speed, right_speed):
        e = vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle, left_speed, vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "SetJointTargetVelocity for left motor got error code")
        e = vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, right_speed, vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "SetJointTargetVelocity for right motor got error code")
        print("Motor speed set to {} {}".format(left_speed, right_speed))

    def get_lidar_data(self):
        (e, state, forceVector, torqueVector) =  vrep.simxReadForceSensor(self.client_id, self.lidar_handle1,vrep.simx_opmode_buffer)
        self.check_error_code(e, "simxReadForceSensor error")
        return e, state, forceVector, torqueVector


    def get_proximity_data(self):
        (e, detectionState, detectedPoint, detectedObjectHandle,
         detectedSurfaceNormalVector) = vrep.simxReadProximitySensor(self.client_id, self.proximity_handle,
                                                                     vrep.simx_opmode_buffer)
        self.check_error_code(e, "simxReadProximitySensor error")
        return detectionState, np.linalg.norm(detectedPoint), detectedSurfaceNormalVector

    def start_simulation(self):
        self.set_motor_speed(0.8, 0.8)
        (errorCode, detectionState, detectedPoint, detectedObjectHandle,
         detectedSurfaceNormalVector) = vrep.simxReadProximitySensor(self.client_id, self.proximity_handle,
                                                                     vrep.simx_opmode_streaming)
        (e, state, forceVector, torqueVector) =  vrep.simxReadForceSensor(self.client_id, self.lidar_handle1,vrep.simx_opmode_streaming)
        self.check_error_code(e, "simxReadForceSensor error")
        while vrep.simxGetConnectionId(self.client_id) != -1:
            is_detected, distance, vector = self.get_proximity_data()
            if is_detected:
               print("distance: {} {}".format(distance, vector))
            print(self.get_lidar_data())
            simulationTime = vrep.simxGetLastCmdTime(self.client_id)
            time.sleep(0.5)

        vrep.simxFinish(self.client_id)


if __name__ == '__main__':
    robot = Robot()
    robot.start_simulation()
