import numpy as np
from utils import Polygon
from global_planner import Sample_search_algorithm, Animate
import time

if __name__ == '__main__':

    ob_list = []
    ob_vertices = [
        np.array([1.71,1.71]),
        np.array([1.71,1.11]),
        np.array([1.11,1.11]),
        np.array([1.11,1.71])
    ]
    ob = Polygon(ob_vertices)
    ob_list.append(ob)

    start = [0,0]
    goal = [1.57,0]
    s = Sample_search_algorithm(step_size=0.3,tolerance=0.1,obs=ob_list,cell_res=[0.2,0.2],
                                additional_optimization_iter=1000)
    start_time = time.time()
    path = s.Search(start,goal)
    end_time = time.time()
    t = end_time - start_time
    print(t)
    a = Animate(path,ob_list)
    a.draw_path() 


