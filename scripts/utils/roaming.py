import numpy as np

#################################### SEEK Algorithm #############################################
class Roaming:
    def __init__(self, max_speed, slowing_speed, slowing_distance) -> None:
        """
        This function initializes the Roaming class
        """
        self.max_speed = max_speed
        self.slowdown_speed = slowing_speed
        self.slowing_distance = slowing_distance
        
    def seek(self, position: np.array, target: np.array, velocity: np.array) -> np.array:
        """
        This function returns the steering velocity to reach the target
        
        :param position: The current position of the robots
        :param target: The target position
        :param velocity: The current velocity of the robots
        :return: The steering velocity
        """
        
        # Vector offset between the target and the current position of the robot
        offset = np.subtract(target, position) 
        
        # At which speed the robot should seek the target.
        desired_velocity = (offset/np.linalg.norm(offset, axis=1)[:,None])*self.max_speed  #First normlize the vector by dividing the vector by its length.
        print(desired_velocity)
        
        # Now if we subtract the desired velocity from the robot's current velocity, we will get the steering velocity.
        steering_velocity = np.subtract(desired_velocity, velocity) # 
        return steering_velocity

    #################################### SEEK Algorithm #############################################


    #################################### Arrive Algorithm #############################################

    def arrival(self, position: np.array, target: np.array, velocity: np.array, slowdown_speed: float, slowing_distance: float) -> np.array:
        """
        This function returns the steering velocity to reach the target

        :param position: The current position of the robots
        :param target: The target position
        :param velocity: The current velocity of the robots
        :param slowdown_speed: The speed at which the robot should start slowing down
        :param slowing_distance: The distance at which the robot should start slowing down
        :return: The steering velocity
        """

        
        # Vector offset between the target and the current position of the robot
        offset = np.subtract(target, position)
        
        #Distance from the target to the robot
        distance = np.linalg.norm(offset)
        
        # If we are not very close to the target 
        if distance > 1:
            
            # Normalize the desired velocity vector
            desired_velocity = (offset/np.linalg.norm(offset, axis=1)[:,None])
            
            # When the slowdown speed is not zero and we are close to the target
            if slowdown_speed > 0 and distance < slowing_distance:
                
                # We reduce the desired velocity
                desired_velocity = desired_velocity*self.max_speed*(distance/slowing_distance)
                
            # Otherwise desired velocity maintains the max speed    
            else:
                desired_velocity = desired_velocity*self.max_speed
            
            # Now if we subtract the desired velocity from the robot's current velocity, we will get the steering velocity.   
            steering_velocity = np.subtract(desired_velocity, velocity)
        
        
        # If we are close to the goal, we nullify the desired velocity 
        else:
            steering_velocity = np.zeros(2)
        
        return steering_velocity

#################################### Arrive Algorithm #############################################
