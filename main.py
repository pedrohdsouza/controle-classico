from control_pid import ControlPID

def main():
    control_pid = ControlPID()

    control_pid.loadmat('TransferFunction14.mat')
    control_pid.set_step()
    control_pid.set_output()
    control_pid.set_time()

    k = control_pid.calculate_k()
    print('k = ', k)

    tau = control_pid.calculate_tau()
    print('tau = ', tau)

    theta = control_pid.calculate_theta(tau)
    print('theta = ', theta)

    Hs = control_pid.transfer_function(k, tau, theta)

    #Fazendo a realimentação
    Hcl = control_pid.feedback(Hs)
    
    erro_malha_aberta = 55.35 - 14
    erro_malha_fechada = 14 - 11.2

    control_pid.plot_output(Hcl)


    kp, ti, td = control_pid.CHR2(k, tau, theta)
    print(kp, ti, td)

    for i in range(5, 15):
        kp = i*0.1
        #kp = 0.6 ou 0.7 é boa
        Chr2 = control_pid.controlador_pid(kp, ti, td, Hs)
        #control_pid.plot_output(Chr2)

    Chr2 = control_pid.controlador_pid(0.7, ti, td, Hs)
    control_pid.plot_output(Chr2)
    kp, ti, td = control_pid.integral_erro(k, tau, theta)
    print(kp, ti, td)

    for i in range(2, 15):
        kp = i*0.1
        #kp = 0.6 ou 0.7 é boa

    integral_erro = control_pid.controlador_pid(0.4, ti, td, Hs)

    control_pid.plot_output(integral_erro)


if __name__=="__main__":
    main()