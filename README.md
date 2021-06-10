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

# The Repository
The repo is a continuation, of [the first thesis repo, on Adises github](https://github.com/ahodzic7/waterlab-estimator). We elected to create a new repo as the old one was a mess, we are not software devs. When running code from this repocitory, we suggest to keep MATLAB at the root folder, in order to avoid problems with referenceing.

The repo containts code for the three most elements of our thesis:
1. System identification:
Code for system identification (Grey-box) of gravity pipes in the [LinedIn](https://www.linkedin.com/in/adis-hodzic-7b2324181/), relying on the physical
laws described by the Saint-Venant hyperbolic Partial Differential
Equations.

2. State estimation:
Full state observability in sewer applications is typically not available,
hence only some subsets of states are measured. 

3. Stochastic Model Predictive Control:
Although standard MPC methods offer a certain degree of robustness,
stochastic MPC is a natural extension to deal with the uncertainties
systematically.

