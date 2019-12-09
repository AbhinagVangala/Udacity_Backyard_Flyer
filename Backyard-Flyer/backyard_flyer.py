#_____________________________UDACITY BACKYARD FLYER PROJECT__________________________#
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        #________________________________STUDENT CODE BEGIN_____________________________ #
        #--------------------------------------------------------------------------------#
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()
        #--------------------------------------------------------------------------------#
        #______________________________STUDENT CODE END__________________________________#

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        #______________________________STUDENT CODE BEGIN________________________________#
        #--------------------------------------------------------------------------------#
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 1.0:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
        #--------------------------------------------------------------------------------#
        #_____________________________SSTUDENT CODE END__________________________________#

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        #____________________________STUDENT CODE BEGIN__________________________________#
        #--------------------------------------------------------------------------------#
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if not self.armed and not self.guided:
                    self.manual_transition()
        #_____________________________STUDENT CODE END____________________________________#
        #---------------------------------------------------------------------------------#

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        #_____________________________STUDENT CODE BEGIN__________________________________#
        #---------------------------------------------------------------------------------#
        print("Square Box Coordinates")
        local_waypoints = [[21.0,0.0,7.0],[21.0,21.0,7.0],[0.0,21.0,7.0,],[0.0,0.0,7.0]]
        return local_waypoints
        #---------------------------------------------------------------------------------#
        #_____________________________STUDENT CODE END____________________________________#


    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        #_____________________________STUDENT CODE BEGIN__________________________________#
        #---------------------------------------------------------------------------------#
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],self.global_position[1],self.global_position[2])
        self.flight_state =States.ARMING
        #---------------------------------------------------------------------------------#
        #_____________________________STUDENT CODE END____________________________________#

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        #_____________________________STUDENT CODE BEGIN__________________________________#
        #---------------------------------------------------------------------------------#
        print("takeoff transition")
        target_altitude = 7.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        #---------------------------------------------------------------------------------#
        #_____________________________STUDENT CODE END____________________________________#

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        #_____________________________STUDENT CODE BEGIN__________________________________#
        #---------------------------------------------------------------------------------#
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop()
        print('target_position',self.target_position)
        self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],0.0)
        self.flight_state = States.WAYPOINT
        #---------------------------------------------------------------------------------#
        #_____________________________STUDENT CODE END____________________________________#

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        #_____________________________STUDENT CODE BEGIN__________________________________#
        #---------------------------------------------------------------------------------#
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING
        #---------------------------------------------------------------------------------#
        #_____________________________STUDENT CODE END____________________________________#

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        #_____________________________STUDENT CODE BEGIN__________________________________#
        #---------------------------------------------------------------------------------#
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING
        #---------------------------------------------------------------------------------#
        #_____________________________STUDENT CODE END____________________________________#

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        #_____________________________STUDENT CODE BEGIN__________________________________#
        #---------------------------------------------------------------------------------#
        print("manual transition")
        self.stop()
        self.in_mission = True
        self.flight_state = States.MANUAL
        #---------------------------------------------------------------------------------#
        #_____________________________STUDENT CODE END____________________________________#

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(3)
    drone.start()
