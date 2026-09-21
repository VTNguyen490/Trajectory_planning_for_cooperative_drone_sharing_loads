import numpy as np

class Polygon():
         
    def __init__(self, vertices):
        
        self.edge = []
        closed_list = []
        # Initialize the first two edges
        
        closed_list.append(0)
        max_angle = 0
        for i in range(1,len(vertices)):
            vector1 = vertices[0]-vertices[i]
            for k in range(1,len(vertices)):
                if i != k:
                    vector2 = vertices[0]-vertices[k]
                    angle = get_angle(vector1,vector2)
                    if angle > np.pi:
                        angle = 2*np.pi - angle
                    if angle > max_angle:
                        max_angle  = angle
                        index1 = i
                        index2 = k

        edge1  = Edge([vertices[0],vertices[index1]])
        edge2  = Edge([vertices[index2],vertices[0]])

        self.edge.append(edge1)
        
        # Connect other edges
        current_index = index1
        current_edge = edge1

        while current_index != index2:
            if not current_index in closed_list:
                max_angle = 0
                for i in range(0,len(vertices)):
                    if not i in closed_list and i!= current_index:
                        vector = vertices[current_index]-vertices[i]
                        angle = get_angle(vector, -current_edge.edge)
                        if angle > np.pi:
                            angle = 2*np.pi - angle
                        if angle > max_angle:
                            max_angle = angle
                            next_index = i

                next_edge = Edge([vertices[current_index],vertices[next_index]])
                self.edge.append(next_edge)
                closed_list.append(current_index)
                # Update for next iteration

                current_edge = next_edge
                current_index = next_index

        self.edge.append(edge2)

        self.vertices = vertices

class Edge():
    
    def __init__(self, vertices):
        
        self.vertices = vertices
        self.edge  = vertices[0] - vertices[1]
        if (self.edge[0] == self.edge[1]) and (self.edge[1] == 0):
            self.normal = None
        else:
            if self.edge[1] != 0:
                a = 1/(1+pow(self.edge[0]/self.edge[1],2))
                a = pow(a,0.5)
                b = -a*self.edge[0]/self.edge[1]
            else:
                b = 1/(1+pow(self.edge[1]/self.edge[0],2))
                b = pow(b,0.5)
                a = -b*self.edge[1]/self.edge[0]
            self.normal = np.array([a,b])

    def __eq__(self, ob):
        return (((self.vertices[0][0] == ob.vertices[0][0] and self.vertices[0][1] == ob.vertices[0][1]) 
                and (self.vertices[1][0] == ob.vertices[1][0] and self.vertices[1][1] == ob.vertices[1][1])) 
        or ((self.vertices[0][0] == ob.vertices[1][0] and self.vertices[0][1] == ob.vertices[1][1])
             and (self.vertices[1][0] == ob.vertices[0][0] and self.vertices[1][1] == ob.vertices[0][1])))


def test_edge_intersect(edge1, edge2):

    if (edge1.vertices[1][0] - edge1.vertices[0][0])*(edge2.vertices[1][0] - edge2.vertices[0][0]) != 0:
        a1 = (edge1.vertices[1][1] - edge1.vertices[0][1])/(edge1.vertices[1][0] - edge1.vertices[0][0])
        b1 = edge1.vertices[1][1]-a1*edge1.vertices[1][0]

        a2 = (edge2.vertices[1][1] - edge2.vertices[0][1])/(edge2.vertices[1][0] - edge2.vertices[0][0])
        b2 = edge2.vertices[1][1]-a2*edge2.vertices[1][0]

        if a1 == a2 and b1 == b2:
            if ((edge1.vertices[0][0]-edge2.vertices[0][0])*(edge1.vertices[0][0]-edge2.vertices[1][0]) <= 0
                or (edge1.vertices[1][0]-edge2.vertices[0][0])*(edge1.vertices[0][0]-edge2.vertices[1][0]) <= 0):
                return True
            else:
                return False
            
        elif a1 == a2 and b1 != b2:
            return False
        else:
            x = -(b2-b1)/(a2-a1)
            if ((x-edge1.vertices[0][0])*(x-edge1.vertices[1][0]) <= 0
            and (x-edge2.vertices[0][0])*(x-edge2.vertices[1][0]) <= 0):
                return True
            else:
                return False
            
    elif (edge1.vertices[1][0] - edge1.vertices[0][0]) == 0 and (edge2.vertices[1][0] - edge2.vertices[0][0]) != 0:
        a2 = (edge2.vertices[1][1] - edge2.vertices[0][1])/(edge2.vertices[1][0] - edge2.vertices[0][0])
        b2 = edge2.vertices[1][1]-a2*edge2.vertices[1][0]

        x = edge1.vertices[0][0]
        y = a2*x+b2

        if ((x-edge2.vertices[0][0])*(x-edge2.vertices[1][0]) <= 0
            and (y-edge1.vertices[0][1])*(y-edge1.vertices[1][1]) <= 0):
            return True
        else:
            return False

    elif (edge1.vertices[1][0] - edge1.vertices[0][0]) != 0 and (edge2.vertices[1][0] - edge2.vertices[0][0]) == 0:
        a1 = (edge1.vertices[1][1] - edge1.vertices[0][1])/(edge1.vertices[1][0] - edge1.vertices[0][0])
        b1 = edge1.vertices[1][1]-a1*edge1.vertices[1][0]

        x = edge2.vertices[0][0]
        y = a1*x+b1

        if ((x-edge1.vertices[0][0])*(x-edge1.vertices[1][0]) <= 0
            and (y-edge2.vertices[0][1])*(y-edge2.vertices[1][1]) <= 0):
            return True
        else:
            return False
    
    else:
        if edge1.vertices[0][0] != edge2.vertices[0][0]:
            return False
        else:
            if ((edge1.vertices[0][1]-edge2.vertices[0][1])*(edge1.vertices[0][1]-edge2.vertices[1][1]) <= 0
                or (edge1.vertices[1][1]-edge2.vertices[0][1])*(edge1.vertices[1][1]-edge2.vertices[1][1]) <= 0):
                return True
            else:
                return False
            
def get_angle(vector1,vector2):
    angle = np.arctan2(vector2[1], vector2[0]) - np.arctan2(vector1[1], vector1[0])
    if angle < 0:
        angle += 2*np.pi
    return angle