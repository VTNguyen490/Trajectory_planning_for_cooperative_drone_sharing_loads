import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from utils import test_edge_intersect, Edge
from local_planer import Optimal_trajectory_generator


class Node:

    def __init__(self,theta1,theta2):
        self.theta1 = theta1
        self.theta2 = theta2

    def addParent(self,parent, cost, u,y):
        self.parent_index = parent
        self.cost_to_parent = cost
        self.input_path = u
        self.output_traj = y

    def getCost(self,node_list):
        d = self.cost_to_parent
        index = self.parent_index
        while index != 0:
            d += node_list[index].cost_to_parent
            index = node_list[index].parent_index         
        return d

    def __eq__ (self,node):
        if (round(self.theta1 - node.theta1,4) == 0 and round(self.theta2 - node.theta2,4) == 0):
            return True
        else:
            return False

def get_length_from_trajectory(traj):
    d = 0
    for i in range(0,len(traj)-3,2):
        d += ((traj[i][0] - traj[i+2][0])**2 + (traj[i+1][0] - traj[i+3][0])**2)**0.5
    return d


## Function to check collision

def collision_trajectory(trajectory,obs):
    for i in range(0,len(trajectory),2):
        current_pose = [trajectory[i][0],trajectory[i+1][0]]
        if collision_pose(current_pose,obs):
            return True
    return False

def collision_pose(pose,obs):
    # Link 1: length 1
    rotational_matrix1 = np.array(
            [np.array([np.cos(pose[0]),-np.sin(pose[0])]),
            np.array([np.sin(pose[0]),np.cos(pose[0])])]
    )
    link1_start_vertice = rotational_matrix1@np.array([[0,0]]).T
    link1_end_vertice = rotational_matrix1@np.array([[1,0]]).T


    link1_edge = Edge([link1_start_vertice,link1_end_vertice])

    # Link 2: length 1
    rotational_matrix2 = np.array(
            [np.array([np.cos(pose[1]),-np.sin(pose[1])]),
            np.array([np.sin(pose[1]),np.cos(pose[1])])]
    )
    link2_start_vertice = rotational_matrix1@rotational_matrix2@np.array([[0,0]]).T + link1_end_vertice
    link2_end_vertice = rotational_matrix1@rotational_matrix2@np.array([[1,0]]).T + link1_end_vertice

    link2_edge = Edge([link2_start_vertice,link2_end_vertice])

    for ob in obs:
        for edge in ob.edge:
            if test_edge_intersect(link1_edge, edge):
                return True
            if test_edge_intersect(link2_edge, edge):
                return True
    return False


## K-nearest neighbor search

def distance(node1,node2):
    return ((node1.theta1 - node2.theta1)**2 + (node1.theta2 - node2.theta2)**2)**0.5

def k_nearest_neighbor_search(node_list,node,k = 5):
    d = distance(node_list[0],node)
    dmin = []
    index_node_min = []
    for i in range(k):
        dmin.append(d)
        index_node_min.append(0)

    for i in range(len(node_list)):
        d = distance(node_list[i],node)
        dmin.append(d)
        index_node_min.append(i)

        index_node_min.pop(dmin.index(max(dmin)))
        dmin.pop(dmin.index(max(dmin)))

    index_node_min = sorted(index_node_min, key = lambda x: dmin[index_node_min.index(x)])
    return index_node_min


## Sampling function

class Sample_search_algorithm:

    def __init__(self, step_size,tolerance,obs,cell_res, additional_optimization_iter):
        self.node_list = []
        self.goal_found = False
        self.iter = 0
        self.step_size = step_size
        self.tolerance = tolerance
        self.obs = obs
        self.cell_res = cell_res
        self.additional_optimization_iter = additional_optimization_iter
        self.RRT_op_iter = 0
        self.goal_index = []
        self.map_full = False
        self.sample_num = 10
        self.op = Optimal_trajectory_generator(self.sample_num)
        self.cell_array = []
        w = int(6.28/cell_res[0])
        h = int(6.28/cell_res[1])
        for i in range(w+1):
            temp = []
            for k in range(h+1):
                temp.append([])
            self.cell_array.append(temp)

    def random_expand(self):
        theta1 = round(random.random()*6.28 ,4)
        theta2 = round(random.random()*6.28 ,4)
        return theta1, theta2

    def bias_expand(self,goal):
        theta1 = goal[0]
        theta2 = goal[1]
        return theta1, theta2


    def cell_array_k_nearest_neighbor_search_2D(self,node,k = 5):

        d = distance(self.node_list[0],node)
        dmin = []
        index_node_min = []
        for i in range(k):
            dmin.append(d)
            index_node_min.append(0)
        
        w = int(node.theta1/self.cell_res[0])
        h = int(node.theta2/self.cell_res[1])

        index_list = []
        offset = 0
        while len(index_list) <= k: 
            if (not ((w-offset) < 0 and (w+offset) > len(self.cell_array)-1)):
                for i in range(-offset,offset +1):
                    if 0 <= h+i and h+i <= len(self.cell_array[0])-1:
                        if w - offset >= 0:
                            for index in self.cell_array[w-offset][h+i]:
                                index_list.append(index)
                        if w + offset <= len(self.cell_array)-1:
                            for index in self.cell_array[w+offset][h+i]:
                                index_list.append(index)
            elif (not ((h-offset) < 0 and (h+offset) > len(self.cell_array[0])-1)):
                for i in range(-offset,offset +1):
                    if 0 <= w+i and w+i <= len(self.cell_array)-1:
                        if h - offset >= 0:
                            for index in self.cell_array[w+i][h-offset]:
                                index_list.append(index)
                        if h + offset <= len(self.cell_array[0])-1:
                            for index in self.cell_array[w+i][h+offset]:
                                index_list.append(index)
            else:
                break
            
            offset += 1

        if index_list:
            for i in range(len(index_list)):
                d = distance(self.node_list[index_list[i]],node)
                dmin.append(d)
                index_node_min.append(index_list[i])
                index_node_min.pop(dmin.index(max(dmin)))
                dmin.pop(dmin.index(max(dmin)))
            index_node_min = sorted(index_node_min, key = lambda x: dmin[index_node_min.index(x)])
        return index_node_min 


    def RRT_star(self,goal):   
        goal_node = Node(goal[0],goal[1])   
        if not self.goal_found:
            if self.iter %10 == 9:
                theta1, theta2 = self.bias_expand(goal)
            else:
                theta1, theta2 = self.random_expand()
        else:
            theta1, theta2 = self.random_expand()
        self.iter += 1
        node = Node(theta1,theta2)
        if self.iter <= 500:
            k_nnear_index = k_nearest_neighbor_search(self.node_list, node)
        else:
            k_nnear_index = self.cell_array_k_nearest_neighbor_search_2D(node)

        nnear_index = k_nnear_index[0]
        nnear = self.node_list[nnear_index]
        d = distance(nnear, node)
        if d > self.step_size:
            vect1 = np.array([nnear.theta1,nnear.theta2])
            vect2 = np.array([node.theta1, node.theta2])
            vect2 = np.round(vect1 + (vect2 - vect1)*self.step_size/d,4)
            node = Node(vect2[0],vect2[1])
        
        theta_start = [nnear.theta1, nnear.theta2]
        theta_end = [node.theta1,node.theta2]

        output_traj, input_traj, cost = self.op.generate_optimal_trajectory(theta_start, theta_end)
        if output_traj.size != 0 and (not collision_trajectory(output_traj,self.obs)):
            self.node_list.append(node)
            node.addParent(nnear_index, cost,input_traj,output_traj)

            w = int(node.theta1/self.cell_res[0])
            h = int(node.theta2/self.cell_res[1])
            self.cell_array[w][h].append(len(self.node_list) - 1)
            if ((node.theta1 - goal_node.theta1)**2 + (node.theta2 - goal_node.theta2)**2)**0.5 <= self.tolerance:
                self.goal_found = True
                self.goal_index.append(len(self.node_list) - 1)
            else:
                if ((self.RRT_op_iter == 10) or self.goal_found):
                    # Additional Optimization
                    self.RRT_op_iter = 0
                    for k in range(1,len(k_nnear_index)):
                        n_index = k_nnear_index[k]
                        nnear = self.node_list[n_index]
                        theta_start = [node.theta1, node.theta2]
                        theta_end = [nnear.theta1,nnear.theta2]

                        output_traj, input_traj,cost = self.op.generate_optimal_trajectory(theta_start, theta_end)

                        nnear_cost = nnear.getCost(self.node_list)
                        node_cost = node.getCost(self.node_list)
                        if output_traj.size != 0 :
                            if node_cost + cost < nnear_cost:
                                if (not collision_trajectory(output_traj,self.obs)):
                                    # Call addParent again to update to new parent node
                                    print(self.node_list[n_index].parent_index)
                                    self.node_list[n_index].addParent(len(self.node_list)-1,cost,input_traj,output_traj)
                                    print(self.node_list[n_index].parent_index)
                                    print("-------------------------")
                else:
                    self.RRT_op_iter += 1

    def Search(self,start,goal):
        node = Node(start[0],start[1])
        input_traj = np.zeros((2*self.sample_num,1))
        output_traj = np.zeros((2*self.sample_num,1))
        for i in range(0,2,len(output_traj)):
            output_traj[i] = start[0]
            output_traj[i+1] = start[1]
        node.addParent(0,0,input_traj,output_traj)
        self.node_list.append(node)
        w = int(node.theta1/self.cell_res[0])
        h = int(node.theta2/self.cell_res[1])
        self.cell_array[w][h].append(len(self.node_list) - 1)
        path = []
        op_iter = self.additional_optimization_iter
        iter = 0
        while iter <= op_iter:
            self.RRT_star(goal)
            if self.goal_found:
                iter += 1
                
        if self.goal_found:
            current_node_index = min(self.goal_index,key = lambda x: self.node_list[x].getCost(self.node_list))
            while current_node_index != 0:
                current_node = self.node_list[current_node_index]
                print(current_node.theta1,current_node.theta2,current_node.getCost(self.node_list), 
                      current_node.input_path[len(current_node.input_path)-2],current_node.input_path[len(current_node.input_path)-1])
                current_node_index = current_node.parent_index
                traj = current_node.output_traj
                for i in range(len(traj)-1,0,-2):
                    pose = [traj[i-1][0],traj[i][0]]
                    path.insert(0,pose)
            return path
        else:
            print("Path not found")
            return None

class Animate:

    def __init__(self, path,obs):

        self.path = path
        self.fig = plt.figure() 
        self.axis = plt.axes(xlim =(-3, 3),
                        ylim =(-3, 3)) 
        self.obstacles = obs
        self.obs = ()
        for obstacle in obs:
            for edge in obstacle.edge:
                x = [edge.vertices[0][0],edge.vertices[1][0]]
                y = [edge.vertices[0][1],edge.vertices[1][1]]
                ob = self.axis.plot(x,y)
        self.line, = self.axis.plot([], [], lw = 2) 
        self.xdata, self.ydata = [], [] 

    def animate(self,fr): 
        i = fr - 1
        rotational_matrix1 = np.array(
            [np.array([np.cos(self.path[i][0]),-np.sin(self.path[i][0])]),
            np.array([np.sin(self.path[i][0]),np.cos(self.path[i][0])])]
        )
        link1_start_vertice = rotational_matrix1@np.array([[0,0]]).T
        link1_end_vertice = rotational_matrix1@np.array([[1,0]]).T

        # Link 2: length 1
        rotational_matrix2 = np.array(
                [np.array([np.cos(self.path[i][1]),-np.sin(self.path[i][1])]),
                np.array([np.sin(self.path[i][1]),np.cos(self.path[i][1])])]
        )
        link2_start_vertice = rotational_matrix1@rotational_matrix2@np.array([[0,0]]).T + link1_end_vertice
        link2_end_vertice = rotational_matrix1@rotational_matrix2@np.array([[1,0]]).T + link1_end_vertice
        
        self.xdata = [link1_start_vertice[0], link1_end_vertice[0],link2_start_vertice[0],link2_end_vertice[0]]
        self.ydata = [link1_start_vertice[1], link1_end_vertice[1],link2_start_vertice[1],link2_end_vertice[1]]
        self.line.set_data(self.xdata, self.ydata) 
        
        return self.line,

    def draw_path(self):   
        anim = animation.FuncAnimation(self.fig, self.animate
                                    ,frames = len(self.path), interval = 40, blit = True)
        writergif = animation.PillowWriter(fps=30)
        anim.save('path.gif',writer=writergif)
    

    



    



        