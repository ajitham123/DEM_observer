# DEM_observer

This MATLAB code performs the state and input estimation for a linear state space system with coloured noise using the Free energy principle from neuroscience. This is based on the paper:

A. A. Meera and M. Wisse, "**Free Energy Principle Based State and Input Observer Design for Linear Systems with Colored Noise**," 2020 American Control Conference (ACC), Denver, CO, USA, 2020, pp. 5052-5058, doi: 10.23919/ACC45564.2020.9147581.

***

In order to simulate it, run DEM_observer.m file.

The *model* structure represents the generative process, while the *brain* structure represents the generative model. 

## List of variable names and their meanings

 * Tt:                Time vector (starting from 0)
 * model.t :          Time vecotor starting from sampling time
 * model.sam_time :   Sampling time
 * model.real_cause :  System input (measured) - nv*nt matrix
 * model.process_x :  States state (only used for plotting) - nt*nx matrix
 * model.process_y :  System output (measured)    - nt*ny matrix
 * model.A        :   A matrix - nx*nx matrix
 * model.B        :   B matrix - nx*nv matrix
 * model.C        :   C matrix - ny*nx matrix
 * model.s        :   Noise smoothness
 * model.p       :   Embedding order of states
 * model.d        :   Embedding order of imputs
 * model.sigma_w  :   Standard deviation of state noise
 * model.sigma_z  :   Standard deviation of observation noise

 * brain.nv        :  Number of inputs
 * brain.ny       :   Number of outputs
 * brain.nx        :  Number of states
 * brain.nt        :  Number of total time steps
