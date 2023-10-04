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
        self.k = None
        self.tau = None
        self.theta = None
        self.hs = None
        self.hcl = None

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
        self.k = float(delta_y / delta_u)
        return self.k

    def calculate_tau(self):
        y_t1 = 0.283*self.output[-1]
        y_t2 = 0.632*self.output[-1]

        closest_from_y_t1 = self.closest(self.output, y_t1)
        closest_from_y_t2 = self.closest(self.output, y_t2)

        self.t1 = np.where(self.output == closest_from_y_t1)[0]
        self.t2 = np.where(self.output == closest_from_y_t2)[0]
        
        self.tau = float(1.5 * (self.t2 - self.t1))
        return self.tau

    def calculate_theta(self):
        self.theta = float(self.t2 - self.tau)
        return self.theta
    
    def closest(self, lst, K):
        lst = np.asarray(lst)
        idx = (np.abs(lst - K)).argmin()
        return lst[idx]
    
    def transfer_function(self):
        num = np.array([self.k])
        den = np.array([self.tau, 1])
        H = cnt.tf(num, den)
        n_pade = 20
        num_pade, den_pade = cnt.pade(self.theta, n_pade)
        H_pade = cnt.tf(num_pade, den_pade )
        self.hs = cnt.series(H , H_pade)
        return self.hs
    
    def feedback(self):
        self.hcl = cnt.feedback(self.hs * 14, 1)
        return self.hcl

    def plot_output(self):
        t = np.linspace(0, 40, 100)
        t, y = cnt.step_response(self.hcl, t)

        plt.plot(self.time.T, self.output, color='r', label='Saída')
        plt.plot(self.time.T, self.step, label='Degrau de entrada')
        plt.plot(t, y, color='g')

        plt.xlabel (' t [ s ] ')
        plt.ylabel('Amplitude')
        plt.title("Controle PID")
        plt.legend(loc="upper left")

        plt.grid()
        plt.show()