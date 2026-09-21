import numpy as np

class Optimal_trajectory_generator:

    def __init__(self, sample_num):
        self.sample_num = sample_num
        self.Q = 5*sample_num*np.eye(2*sample_num)
        self.R = np.eye(2*sample_num)
        self.sample_num = sample_num
        self.delta_T = 1/sample_num
        W1 = np.zeros((sample_num*2,sample_num*2))
        W2 = np.zeros((sample_num*2,sample_num*2))
        M = np.zeros((sample_num*2,sample_num*2))
        O = np.zeros((sample_num*2,2))
        H = np.zeros((sample_num*2,2))
        for i  in range(self.sample_num):
            W1[i*2:(i+1)*2:1,i*2:(i+1)*2:1] = np.eye(2)
            W2[i*2:(i+1)*2:1,i*2:(i+1)*2:1] = np.eye(2)
            if i != 0:
                W1[i*2:(i+1)*2:1,(i-1)*2:i*2:1] = -np.eye(2)
                W2[i*2:(i+1)*2:1,(i-1)*2:i*2:1] = -np.eye(2)

            O[i*2:(i+1)*2:1,0:2:1] = np.eye(2)
            H[i*2:(i+1)*2:1,0:2:1] = self.delta_T*np.eye(2)
            for k in range(i+1):
                M[i*2:(i+1)*2:1,k*2:(k+1)*2:1] = self.delta_T*np.eye(2)
        self.Q_prime = W1.T @ self.Q @ W1
        self.R_prime = W2.T @ self.R @ W2
        self.O = O
        self.H = H
        self.M =  M
        self.A = (self.M.T @ self.Q_prime @ self.M) + self.R_prime 
        self.A = np.concatenate((self.A,self.H),axis = 1)
        temp = np.concatenate((self.H.T,np.zeros((2,2))),axis = 1)
        self.A = np.concatenate((self.A,temp),axis = 0)


    def generate_optimal_trajectory(self,theta_start, theta_end):
        sample_num = self.sample_num
        theta_0 = np.array([[theta_start[0]],[theta_start[1]]])
        Q_1 = self.Q[0:2:1,0:2:1]
        a = np.array([[theta_end[0]],[theta_end[1]]]) - theta_0
        A = self.A

        if np.linalg.matrix_rank(A) == len(A): # Only proceed if A is full rank (inversable)      
            B = np.zeros(((sample_num+1)*2,1))
            B[0:sample_num*2:1] -= self.M.T @ self.Q_prime @ self.O @ theta_0
            B[0:2:1] += self.delta_T * (Q_1 @ theta_0)
            B[sample_num*2:sample_num*2+2:1] += a

            output = np.linalg.inv(A) @ B
            u = output[0:sample_num*2:1]

            # Forward calculating the trajectory
            state = self.O @ theta_0 + self.M @ u

            # Calculate the cost
            # print(Q_1)
            # print(state.T @ self.Q_prime @ state)
            # print(u.T @ self.R_prime @ u)
            # print(theta_0.T @ Q_1 @ state[0:2:1])
            # print(u[0:2:1].T @ R_0 @ u_minus1)
            
            cost = (state.T @ self.Q_prime @ state + u.T @ self.R_prime @ u - theta_0.T @ Q_1 @ state[0:2:1]
             - state[0:2:1].T @ Q_1 @ theta_0 + theta_0.T @ Q_1 @ theta_0)
            cost = cost[0][0]
            # print(cost)
            # print("------------------------------------------")
            return state, u, cost
        
        else:
            return np.array([]), np.array([]), None
