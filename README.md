# The Repositorys History
The repo is a continuation, of [the first thesis repo, on Adises github](https://github.com/ahodzic7/waterlab-estimator). We elected to create a new repo as the old one was a mess; we are not software devs. When running code from this repository, we suggest keeping MATLAB at the root folder to avoid problems with referencing. I small picture of the lab is shown in the figure below.

<img src="https://user-images.githubusercontent.com/25584836/121507470-f2e8c680-c9e4-11eb-8a63-db0115255e29.png" width="50%" height="50%">

# Content of the Repository
The repo containts code for the three most elements of our thesis:
**1. System identification:**
Code for system identification (Grey-box) of gravity pipes is located in the folder of the same name. The gravity pipe identication, relies on a model derived based on the physical
laws described by the Saint-Venant hyperbolic Partial Differential Equations. Code for estiating the different kinds of process noise present in the [the AAU Smart Water Lab](https://vbn.aau.dk/da/equipments/smart-water-infrastructures-laboratory-swil#:~:text=The%20AAU%20Smart%20Water%20Infrastructures,Collection%20or%20District%20Heating%20Systems) has been included in this folder aswell.

**2. State estimation:**
Full state observability in sewer applications is typically not available. Therefore we elected to create a simple Kalman filter estimating pipestates in a 8 pipe state model with 4 sensors. The code for this is includeded in the Kalman_Filter folder. In that folder a short demo script is included.

**3. Stochastic Model Predictive Control:**
As for the control of the [the AAU Smart Water Lab](https://vbn.aau.dk/da/equipments/smart-water-infrastructures-laboratory-swil#:~:text=The%20AAU%20Smart%20Water%20Infrastructures,Collection%20or%20District%20Heating%20Systems) we used a simulink framework provieded by AAU. Alot of the experiments code is specific to the lab. This means that if code applicable to other projects is the only code of interest attentions should only be paid to the server side files called: ..._full_DW_real.m and ..._init_DW_real.m

**Note:** that all simulink documents in the repo are setup such that they work for the configuration of [the AAU Smart Water Lab](https://vbn.aau.dk/da/equipments/smart-water-infrastructures-laboratory-swil#:~:text=The%20AAU%20Smart%20Water%20Infrastructures,Collection%20or%20District%20Heating%20Systems).

# Stochastic Model Predictive Control of Combined Sewer Overflows in Sanitation Networks
**Thesis Abstract:**
Sanitation networks are vital infrastructure in modern society. They are used for transporting wastewater and rainwater from cities to treatment facilities, where wastewater is treated before being released into the environment. Most countries still use combined sanitation networks where wastewater and rainwater are transported in a single pipe. This leaves the combined sanitation network prone to overflow in the event of heavy rainfall. A solution to minimizing the overflow is Real Time Control (RTC).   A popular state-of-the-art RTC method used to anticipate and minimize overflows is standard Model Predictive Control (MPC). However, the standard formulation of the MPC does face challenges when dealing with the uncertainty caused by the inflow disturbances, i.e., the weather forecasts.
In order to better handle the uncertainties, we propose an extended model predictive framework called Chance-Constrained MPC (CC-MPC). First, the nominal multi-objective MPC is formulated to deal with the challenges in the sanitation network. Then, the framework is extended to our stochastic MPC formulation. Two controllers are compared in a laboratory emulation of the network subsystem that we call the Two Tank Topology. Gravity pipe elements determine the primary dynamics that define the transport of wastewater through a network. Both controller frameworks require a model that can capture gravity pipe dynamics in order to predict overflow. Therefore, we developed a linear Diffusion Wave model based on the discretized Saint-Venant partial differential equations. The model is validated through a data-driven parameter estimation framework. Identification is conducted in a real network simulation and in the real-life experimental setup created in AAU Smart Water Lab.

The full thesis can be found in the AAU project liberay: **Link will be provided after the examn date**

**Thesis Supervisors:**
- Krisztian Mark Balla - [LinkedIn](https://www.linkedin.com/in/krisztian-mark-balla-ba6484191/)
- Carsten Skovmose Kallesøe - [LinedIn](https://www.linkedin.com/in/carsten-skovmose-kalles%C3%B8e-97b5865/)

**Names of Authors:**
- Adis Hodzic - [LinedIn](https://www.linkedin.com/in/adis-hodzic-7b2324181/)
- Casper Houtved Knudsen - [LinedIn](https://www.linkedin.com/in/thecasperknudsen/)

