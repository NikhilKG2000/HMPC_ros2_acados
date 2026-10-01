# mpc-sdp
This repository implements the offline computation of the reference tracking MPC formulation: computing terminal ingredients using an SDP. The formulation is implemented in a modular way, so it can easily be adjusted for other types of nonlinear system models. Refer to [How to customize?](#how-to-customize) for more information.

Contents:\
[Install](#install)\
[Run](#run)\
[How to customize?](#how-to-customize)




## Install
To run the code in this repository, we use [MATLAB R2023a](https://nl.mathworks.com/products/new_products/release2023a.html) with the following packages:
- [CasADi](https://web.casadi.org/get)
- [YALMIP](https://yalmip.github.io) via [tbxmanager](https://tbxmanager.com)
- [SDPT3](https://blog.nus.edu.sg/mattohkc/softwares/sdpt3)

> :information_source: The terminal ingredients are already computed and saved as *offline_comps_tracking.mat* in this repository. These packages are only required if you want to change the offline design.




## Run
1. Run [*solve_sdp.m*](./solve_sdp.m) to solve the SDP and store the results in [*P_K_tracking.mat*](./P_K_tracking.mat).
2. Run [*compute_c_alpha.m*](./compute_c_alpha.m) to compute constants $c_j^\mathrm{s}, j \in \mathbb{N}_{[1,n^\mathrm{s}]}$, $c^\mathrm{o}$, and terminal set scaling $\alpha$ and store the results in [*offline_comps_tracking.mat*](./offline_comps_tracking.mat).
    > :bulb: You can play around with the minimum distance between the trajectory and obstacles $d$ in [*compute_c_alpha.m*](./compute_c_alpha.m) to see how the terminal set scaling $\alpha$ changes.

Done!



## How to customize?
Interested in trying out this method for your own nonlinear system? Easy! Follow these steps:

1. Implement your model based on [*HovergamesModel*](./HovergamesModel.m).
2. Change the lines `model = <your_model_name>`, `Q = [...]` and `R = [...]` in [*solve_sdp.m*](./solve_sdp.m).

That's it! Now you can run the SDP for your own system model :)

> :bulb: For non-holonomic systems, such as cars, it is necessary to enforce a minimum non-zero velocity.
