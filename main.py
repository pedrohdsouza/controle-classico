from control_pid import ControlPID
import numpy as np
import control as cnt
import matplotlib.pyplot as plt

class Main():
    def __init__(self):
        self.output = None
        self.step = None
        self.time = None
        self.k = None
        self.tau = None
        self.theta = None

    def run(self):
        control_pid = ControlPID()

        Hs_14 = control_pid.loadmat('TransferFunction14.mat')
        self.step = control_pid.set_step()
        self.output = control_pid.set_output()
        self.time = control_pid.set_time()

        self.k = control_pid.calculate_k()
        self.tau = control_pid.calculate_tau()
        self.theta = control_pid.calculate_theta(self.tau)

        Hs = control_pid.transfer_function(self.k, self.tau, self.theta)

        Hcl = control_pid.feedback(Hs)
        
        erro_malha_aberta = abs(55.35 - 14)
        erro_malha_fechada = abs(11.2 - 14)

        #control_pid.plot_output(Hcl)

        kp, ti, td = control_pid.CHR2(self.k, self.tau, self.theta)
        print(kp, ti, td)

        for i in range(5, 15):
            kp = i*0.1
            #kp = 0.6 ou 0.7 é boa
            Chr2 = control_pid.controlador_pid(kp, ti, td, Hs)
            #control_pid.plot_output(Chr2)

        Chr2 = control_pid.controlador_pid(0.7, ti, td, Hs)
        #control_pid.plot_output(Chr2)
        kp, ti, td = control_pid.integral_erro(self.k, self.tau, self.theta)
        print(kp, ti, td)

        for i in range(2, 15):
            kp = i*0.1
            #kp = 0.6 ou 0.7 é boa

        integral_erro = control_pid.controlador_pid(0.4, ti, td, Hs)

        #control_pid.plot_output(integral_erro)

        t = np.linspace(0, 40, 100)
        t, y = cnt.step_response(Hs * self.step[0], t)

        plt.subplot(2, 3, 1)
        plt.plot(self.time.T, self.output, color='r', label='Saída')
        plt.plot(self.time.T, self.step, label='Degrau de entrada')
        plt.xlabel (' t [ s ] ')
        plt.ylabel('Amplitude')
        plt.grid()

        plt.subplot(2, 3, 2)
        plt.plot(self.time.T, self.output, color='r', label='Saída')
        plt.plot(self.time.T, self.step, label='Degrau de entrada')
        plt.plot(t, y, color='b')
        plt.xlabel (' t [ s ] ')
        plt.ylabel('Amplitude')
        plt.title('Malha Aberta')
        plt.grid()

        t, y = cnt.step_response(Hcl * self.step[0], t)

        plt.subplot(2, 3, 3)
        plt.plot(self.time.T, self.output, color='r', label='Saída')
        plt.plot(self.time.T, self.step, label='Degrau de entrada')
        plt.plot(t, y, color='y')
        plt.xlabel (' t [ s ] ')
        plt.ylabel('Amplitude')
        plt.title('Malha Fechada')
        plt.grid()

        t, y = cnt.step_response(Chr2 * self.step[0], t)
        plt.subplot(2, 3, 4)
        plt.plot(self.time.T, self.output, color='r', label='Saída')
        plt.plot(self.time.T, self.step, label='Degrau de entrada')
        plt.plot(t, y, color='c')
        plt.xlabel (' t [ s ] ')
        plt.ylabel('Amplitude')
        plt.title('Método CHR2')
        plt.grid()

        t, y = cnt.step_response(integral_erro * self.step[0], t)
        plt.subplot(2, 3, 5)
        plt.plot(self.time.T, self.output, color='r', label='Saída')
        plt.plot(self.time.T, self.step, label='Degrau de entrada')
        plt.plot(t, y, color='m')
        plt.xlabel (' t [ s ] ')
        plt.ylabel('Amplitude')
        plt.title('Método Integral do Erro')
        plt.grid()

        plt.suptitle("Controle PID")
        plt.legend(loc="upper left")
        plt.show()

    def print_menu(self):
        print('--- Menu ---')
        print()

        menu_options = {
        1: 'Gerar dados de controle grupo 14',
        2: 'Entrar com kp, ti e td para calcular controle CHR2',
        3: 'Exit',
        }

        for key in menu_options.keys():
            print (key, '--', menu_options[key] )

        print()
    
    def menu(self):
        while(True):
            print()
            self.print_menu()
            option = ''

            try:
                option = int(input('Escolha uma opção: '))
                print()
            except:
                print('Entrada incompatível. Você deve escolher uma opção existente...')
            
            if option == 1:
                self.run()
            if option == 2:
                kp = input('Entre com o valor de kp: ')
                ti = input('Entre com o valor de ti: ')
                td = input('Entre com o valor de td: ')
            elif option == 3:
                print('Saindo...')
                exit()


if __name__ == "__main__":
    main = Main()
    main.menu()