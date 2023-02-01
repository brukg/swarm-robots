import numpy as np
from utils.fov import field_of_view
class FormationControl:
    def __init__(self, pose, leader_index, num_robots, formation_structure, fov_radius, behaviors, fov_radius_avoidance = 0.2):
        print("Formation Control")
        self.num_robots = num_robots
        self.leader_index = leader_index
        self.formation_structure = formation_structure
        self.fov_radius = fov_radius
        self.fov_radius_avoidance = fov_radius_avoidance
        self.velocity = np.zeros((num_robots, 2))
        self.pose = pose
        self.behaviors = behaviors

    # get adjacency matrix
    def get_adjacency_matrix(self, neighbour_index, num_robots, leader_index):
        
        adjacency_matrix = np.zeros((num_robots, num_robots))

        for i in range(num_robots):
            for j in range(num_robots):
                if i == j:
                    continue
                else:
                    if j in neighbour_index[i][0]:
                        adjacency_matrix[i,j] = 1

        adjacency_matrix[leader_index,:] = np.zeros((adjacency_matrix.shape[1]))

        return adjacency_matrix
    
    def set_structure(self, formation_structure):
        print("set structure")
        self.formation_structure = formation_structure
    # formation algorithm
    def formation(self):
        neighbour_index, _ = field_of_view(self.pose, self.num_robots, self.fov_radius)
        # print(neighbour_index)
        adjacency_matrix = self.get_adjacency_matrix(neighbour_index, self.num_robots, self.leader_index)

        # print(adjacency_matrix)
        # print("formation", self.formation_structure)
        for i in range(self.num_robots):
            self.velocity[i,:] = [0,0]
            for j in range(self.num_robots):
                temp_velocity = adjacency_matrix[i,j]*((self.pose[j,0:2] - self.pose[i,0:2]) - (self.formation_structure[j,:] - self.formation_structure[i,:]))
                # print(temp_velocity)
                self.velocity[i,:] = self.velocity[i,:] + temp_velocity

        # add seperation velocity
        _, neighbour_list_avoidance = field_of_view(self.pose, self.num_robots, self.fov_radius_avoidance)
        sep = self.behaviors.seperation(self.pose, neighbour_list_avoidance, 8)[:,:2]
        sep[self.leader_index,:] = [0,0]
        self.velocity = self.velocity + sep 
        
        return self.velocity
