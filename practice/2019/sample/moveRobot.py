import vrep
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 20
LIDAR_DEVICE            = '/dev/ttyACM0'
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import URG04LX as LaserModel
#from breezylidar import URG04LX as Lidar

from roboviz import MapVisualizer


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
            return False
        return True

    def set_motor_speed(self, left_speed, right_speed):
        e = vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor_handle, left_speed, vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "SetJointTargetVelocity for left motor got error code")
        e = vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor_handle, right_speed, vrep.simx_opmode_oneshot_wait)
        self.check_error_code(e, "SetJointTargetVelocity for right motor got error code")
        print("Motor speed set to {} {}".format(left_speed, right_speed))

    def get_lidar_data(self):
        point_data, dist_data = [], []
        #e, data = vrep.simxGetStringSignal(self.client_id, "lidarMeasuredData", vrep.simx_opmode_buffer)
        #if self.check_error_code(e, "simxGetStringSignal points error"):
        #    measuredData = vrep.simxUnpackFloats(data)
        #    point_data = np.reshape(measuredData, (int(len(measuredData) / 3), 3))

        e, data = vrep.simxGetStringSignal(self.client_id, "lidarMeasuredData", vrep.simx_opmode_buffer)
        if self.check_error_code(e, "simxGetStringSignal lidar distances error"):
            buf = data.split("!#!")
            if len(buf) >= 2:
                dist_data = vrep.simxUnpackFloats(buf[1])
                measuredData = vrep.simxUnpackFloats(buf[0])
                point_data = np.reshape(measuredData, (int(len(measuredData) / 3), 3))

        return e, point_data, dist_data


    def get_proximity_data(self):
        (e, detectionState, detectedPoint, detectedObjectHandle,
         detectedSurfaceNormalVector) = vrep.simxReadProximitySensor(self.client_id, self.proximity_handle,
                                                                     vrep.simx_opmode_buffer)
        self.check_error_code(e, "simxReadProximitySensor error")
        return detectionState, np.linalg.norm(detectedPoint), detectedSurfaceNormalVector

    def run_SLAM(self, scan_data):
        # Create an RMHC SLAM object with a laser model and optional robot model
        slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        # Set up a SLAM display
        viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')
        # Initialize empty map
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        while True:
            # Update SLAM with current Lidar scan
            slam.update(scan_data)
            # Get current robot position
            x, y, theta = slam.getpos()
            # Get current map bytes as grayscale
            slam.getmap(mapbytes)
            # Display map and robot pose, exiting gracefully if user closes it
            if not viz.display(x/1000., y/1000., theta, mapbytes):
                exit(0)

    def start_simulation(self):
        self.set_motor_speed(0.8, 0.0)
        (errorCode, detectionState, detectedPoint, detectedObjectHandle,
         detectedSurfaceNormalVector) = vrep.simxReadProximitySensor(self.client_id, self.proximity_handle,
                                                                     vrep.simx_opmode_streaming)
        e, data = vrep.simxGetStringSignal(self.client_id, "lidarMeasuredData", vrep.simx_opmode_streaming)
        self.check_error_code(e, "simxGetStringSignal lidar error")

        # Create an RMHC SLAM object with a laser model and optional robot model
        slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        # Set up a SLAM display
        viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')
        # Initialize empty map
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

        while vrep.simxGetConnectionId(self.client_id) != -1:
            is_detected, distance, vector = self.get_proximity_data()
            if is_detected:
               print("distance: {} {}".format(distance, vector))
            e, lidar_data, dist_data = self.get_lidar_data()
            point_data_len = len(lidar_data)
            dist_data_len = len(dist_data)
            print("shape: {}; data: {}".format(point_data_len, dist_data_len))

            # Update SLAM , with current Lidar scan
            slam.update(dist_data[0:-2])
            # Get current robot position
            x, y, theta = slam.getpos()
            print("x: {}; y: {}; theta: {}".format(x, y, theta))
            # Get current map bytes as grayscale
            slam.getmap(mapbytes)
            # Display map and robot pose, exiting gracefully if user closes it
            if not viz.display(x/1000., y/1000., theta, mapbytes):
                exit(0)

            simulationTime = vrep.simxGetLastCmdTime(self.client_id)
            time.sleep(0.1)

        vrep.simxFinish(self.client_id)


if __name__ == '__main__':
    robot = Robot()
    robot.start_simulation()
