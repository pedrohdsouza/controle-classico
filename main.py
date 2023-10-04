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

    print('Hs = ', Hs)

    Hcl = control_pid.feedback(Hs)
    print('Hcl = ', Hcl)

    control_pid.plot_output(Hcl)

if __name__=="__main__":
    main()