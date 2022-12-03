import numpy as np

class ObstacleAvoidance:

    # Constructor
    def __init__(self, r, distance=0.2, fov=np.pi/4, max_see_ahead=0.5):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None

        self.there_is_map = False
        self.num_robots = r
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        self.ahead = np.zeros((r,2))
        self.ahead_l = np.zeros((r,2))
        self.ahead_r = np.zeros((r,2))
        self.fov = np.deg2rad(fov)
        self.max_see_ahead = max_see_ahead

        self.obstacle_center = np.zeros((r,2))
        self.avoidance_vector = np.zeros((r,2))

    # Set occupancy map, its resolution and origin. 
    def set_map(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        print("Map set", self.map.shape)
        print(self.map[-2, -2])
        
    def look_ahead(self, pose, vel):
        """Given a pose and a velocity, predicts the position of the robot ahead and check if it is obstacle free.
        
        Args:
            pose (np.array): 2D pose of the robots.
            vel (np.array): 2D velocity of the robots.
            """

        # self.ahead[:,0] = pose[:,0] + (vel[:,0]*np.cos(vel[:,2]) + vel[:,1]*np.cos(vel[:,2] +np.pi/2) ) * MAX_SEE_AHEAD
        # self.ahead[:,1] = pose[:,1] + (vel[:,0]*np.sin(vel[:,2] + vel[:,1]*np.sin(vel[:,2] +np.pi/2) )) * MAX_SEE_AHEAD


        #reset vector to zero
        self.avoidance_vector = np.zeros((self.num_robots,2))

        #look ahead for each robot x, y
        self.ahead[:,0] = pose[:,0] + vel[:,0] * self.max_see_ahead
        self.ahead[:,1] = pose[:,1] + vel[:,1] * self.max_see_ahead

        # line vector pointing to the front of the robot
        L = self.ahead - pose

        #get length of line for each robot
        L_norm = np.linalg.norm(self.ahead - pose, axis=1)


        #angle between the line vector and the x axis
        alpha = np.arctan2(L[:,1], L[:,0])


        #rotate the line vector by 30 degrees
        self.ahead_l[:,0] = pose[:,0] + L_norm*np.cos(alpha + self.fov)
        self.ahead_l[:,1] = pose[:,1] +L_norm*np.sin(alpha + self.fov)

        #rotate the line vector by -30 degrees
        self.ahead_r[:,0] = pose[:,0] + L_norm*np.cos(alpha - self.fov)   
        self.ahead_r[:,1] = pose[:,1] + L_norm*np.sin(alpha - self.fov)
        if self.there_is_map:

            for i in range(0, self.num_robots):
                
                # print("pose", i, pose[i,:])
                # print("ahead", self.ahead[i,:])


                if not self.is_valid(self.ahead[i,:]) or not self.is_valid(self.ahead_l[i,:])  or not self.is_valid(self.ahead_r[i,:]):
                    # self.ahead_l[i,0] = pose[i,0] + (vel[i,0]*np.cos(vel[i,2]) + vel[i,1]*np.cos(vel[i,2] +np.pi/2) ) * MAX_SEE_AHEAD
                    # self.ahead_l[i,1] = pose[i,1] + (vel[i,1]*np.sin(vel[i,2] + vel[i,1]*np.sin(vel[i,2] +np.pi/2) )) * (MAX_SEE_AHEAD*2)

                    # self.ahead_r[i,0] = pose[i,0] + (vel[i,0]*np.cos(vel[i,2]) + vel[i,1]*np.cos(vel[i,2] +np.pi/2) ) * (MAX_SEE_AHEAD*2)
                    # self.ahead_r[i,1] = pose[i,1] + (vel[i,1]*np.sin(vel[i,2] + vel[i,1]*np.sin(vel[i,2] +np.pi/2) )) * MAX_SEE_AHEAD
                    # print("Obstacle ahead ")

                    # print("ahead valid",  self.is_valid(self.ahead[i,:]))
                    # print("left valid ", self.is_valid(self.ahead_l[i,:]))
                    # print("right valid", self.is_valid(self.ahead_r[i,:]))
                    if not self.is_valid(self.ahead[i,:])  and \
                       not self.is_valid(self.ahead_l[i,:])  and \
                       not self.is_valid(self.ahead_r[i,:]):

                        self.avoidance_vector[i,0] =  pose[i,0] - self.ahead[i,0]
                        self.avoidance_vector[i,1] =  pose[i,1] - self.ahead[i,1]
                        self.avoidance_vector[i,:] = self.avoidance_vector[i,:]/np.linalg.norm(self.avoidance_vector[i,:])
                    elif not self.is_valid(self.ahead_l[i,:]) and self.is_valid(self.ahead_r[i,:]):
                        # print("Obstacle ahead left")
                        self.avoidance_vector[i,0] =  self.ahead_r[i,0] - self.ahead_l[i,0]
                        self.avoidance_vector[i,1] =  self.ahead_r[i,1] - self.ahead_l[i,1]
                        self.avoidance_vector[i,:] = self.avoidance_vector[i,:]/np.linalg.norm(self.avoidance_vector[i,:])

                    elif not self.is_valid(self.ahead_r[i,:]) and self.is_valid(self.ahead_l[i,:]):
                        # print("Obstacle ahead right")
                        # self.obstacle_center[i,0] = self.ahead_r[i,0]
                        # self.obstacle_center[i,1] = self.ahead_r[i,1]
                        self.avoidance_vector[i,0] =  self.ahead_l[i,0] - self.ahead_r[i,0]
                        self.avoidance_vector[i,1] =  self.ahead_l[i,1] - self.ahead_r[i,1]
                        self.avoidance_vector[i,:] = self.avoidance_vector[i,:]/np.linalg.norm(self.avoidance_vector[i,:])
                    else:
                        # print("No obstacle ahead")
                        self.avoidance_vector = np.zeros((self.num_robots, 2))
            # print("avoidance_vector", self.avoidance_vector)

            return self.avoidance_vector

    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose, ): 
        """Given a pose, returs true if the pose is not in collision and false othewise.
        
        Args:
            pose (np.array): 2D pose of the robot.
        """

        p = self.__position_to_map__(np.asarray((pose[0], pose[1]))) # : convert world robot position to map coordinates using method __position_to_map__

        
        vicinity = int(self.distance/self.resolution)
        
        x_width = self.map.shape[0]; y_length = self.map.shape[1]
          
        if len(p) == 2: # if p is outside the map return true (unexplored positions are considered free)
            u_min = p[0] - vicinity if p[0] - vicinity  > 0 else 0 # x min
            v_min = p[1] - vicinity if p[1] - vicinity  > 0 else 0 # y min
            u_max = p[0] + vicinity if p[0] + vicinity  < x_width - 1 else x_width  # x max
            v_max = p[1] + vicinity if p[1] + vicinity  < y_length - 1 else y_length # y max
            if np.any(self.map[u_min:u_max, v_min:v_max] > 0):
                return False            # Obstacle
        return True
    

    def __position_to_map__(self, p): # P has to be a numpy array
        """Transform position with respect the map origin to cell coordinates
        
        Args:
            p (np.array): 2D position with respect the map origin."""
        # convert world position to map coordinates
        uv = (p - self.origin) / self.resolution

        # keep position inside map
        if uv[0] < 0 or uv[0] >= self.map.shape[0] or uv[1] < 0 or uv[1] >= self.map.shape[1]:
            return []
        return uv.astype(int)
    