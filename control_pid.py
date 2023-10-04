from scipy.io import loadmat
import numpy as np
import control as cnt
import matplotlib.pyplot as plt

class ControlPID:
    def __init__(self) -> None:
        self.mat = None
        self.step = None
        self.output = None
        self.time = None
        self.t1 = None
        self.t2 = None

    def loadmat(self, transfer_function):
        self.mat = loadmat(transfer_function)
    
    def set_step(self):
        self.step = self.mat.get('degrau')
    
    def set_output(self):
        self.output = self.mat.get('saida')
    
    def set_time(self):
        self.time = self.mat.get('t')

    def calculate_k(self):
        delta_y = self.output[-1]
        delta_u = self.step[0]
        return float(delta_y / delta_u)

    def calculate_tau(self):
        #print('Saida = ', self.output[-1])

        y_t1 = 0.283*self.output[-1]
        y_t2 = 0.632*self.output[-1]

        #print('y(t1) 0.283*saida = ', y_t1)
        #print('y(t2) 0.632*saida = ', y_t2)

        closest_from_y_t1 = self.closest(self.output, y_t1)
        closest_from_y_t2 = self.closest(self.output, y_t2)

        self.t1 = self.time[0][int(np.where(self.output == closest_from_y_t1)[0])]
        self.t2 = self.time[0][int(np.where(self.output == closest_from_y_t2)[0])]

        #print('t1 = ', self.t1)
        #print('t2 = ', self.t2)
        
        return float(1.5 * (self.t2 - self.t1))

    def calculate_theta(self, tau):
        return float(self.t2 - tau)
    
    def closest(self, lst, K):
        lst = np.asarray(lst)
        idx = (np.abs(lst - K)).argmin()
        return lst[idx]
    
    def transfer_function(self, k, tau, theta):
        num = np.array([k])
        den = np.array([tau, 1])
        H = cnt.tf(num, den)
        n_pade = 20
        num_pade, den_pade = cnt.pade(theta, n_pade)
        H_pade = cnt.tf(num_pade, den_pade)
        return cnt.series(H , H_pade)
    
    def feedback(self, Hs):
        return cnt.feedback(Hs * 14, 1)

    def plot_output(self, Hcl):
        t = np.linspace(0, 40, 100)
        t, y = cnt.step_response(Hcl, t)

        plt.plot(self.time.T, self.output, color='r', label='Saída')
        plt.plot(self.time.T, self.step, label='Degrau de entrada')
        plt.plot(t, y, color='g', label='Malha fechada')

        plt.xlabel (' t [ s ] ')
        plt.ylabel('Amplitude')
        plt.title("Controle PID")
        plt.legend(loc="upper left")

        plt.grid()
        plt.show()