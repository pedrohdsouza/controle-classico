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
        return self.mat
    
    def set_step(self):
        self.step = self.mat.get('degrau')
        print(len(self.step))
        return self.step
    
    def set_output(self):
        self.output = self.mat.get('saida')
        return self.output
    
    def set_time(self):
        self.time = self.mat.get('t')
        return self.time

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
        return cnt.feedback(Hs, 1)
    
    def CHR2(self, k, tau, theta):
        kp = 0.95*tau/(k*theta)
        ti = 1.357*tau
        td = 0.473*theta

        return kp, ti, td
    
    def integral_erro(self, k, tau, theta):
        kp = (1 / (theta/tau) + 0.2) / k
        ti = ((0.3 * (theta / tau) + 1.2) / ((theta / tau) + 0.08)) * theta
        td = (1 / (90 * (theta / tau))) * theta

        return kp, ti, td
    
    def controlador_pid(self, kp, Ti, Td, Hs):
        # Controlador proporcional
        numkp = np.array([kp])
        denkp = np.array([1])
        #integral
        numki = np.array([kp])
        denki = np.array([Ti,0])
        #derivativo
        numkd = np.array([kp*Td,0])
        denkd = np.array([1])

        #Construindo o controlador PID
        Hkp = cnt.tf(numkp, denkp)
        Hki=cnt.tf(numki, denki)
        Hkd=cnt.tf(numkd, denkd)
        Hctrl1 = cnt.parallel(Hkp, Hki)
        Hctrl = cnt.parallel(Hctrl1 , Hkd)
        Hdel = cnt.series(Hs , Hctrl)
        #Fazendo a realimentação
        Hcl = cnt.feedback(Hdel, 1)
        return Hcl

    def plot_output(self, transfer_function):
        t = np.linspace(0, 40, 100)
        t, y = cnt.step_response(transfer_function * self.step[0], t)

        plt.subplot(1, 2, 1)
        plt.plot(self.time.T, self.output, color='r', label='Saída')
        plt.plot(self.time.T, self.step, label='Degrau de entrada')
        plt.plot(t, y, color='b', label='Malha Fechada')

        plt.xlabel (' t [ s ] ')
        plt.ylabel('Amplitude')
        plt.title("Controle PID")
        plt.legend(loc="upper left")

        plt.grid()
        plt.show()