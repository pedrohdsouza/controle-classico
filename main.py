from control_pid import ControlPID

def main():
    control_pid = ControlPID()

    control_pid.loadmat('TransferFunction14.mat')
    control_pid.set_step()
    control_pid.set_output()
    control_pid.set_time()

    k = control_pid.calculate_k()
    print(k)

    tau = control_pid.calculate_tau()
    print(tau)

    theta = control_pid.calculate_theta()
    print(theta)

    control_pid.transfer_function()
    control_pid.feedback()

    control_pid.plot_output()

    #control_pid.plot_feedback()

if __name__=="__main__":
    main()