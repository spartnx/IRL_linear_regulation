# IRL Policy Iteration for Adaptive Optimal Regualtion of Continuous-Time Linear Systems
Matlab implementation of the Integral Reinforcement Learning (IRL) Policy Iteration algorithm for the adaptive optimal regulation of continuous-time linear systems developed by Vrabie et al. [1]
The dynamic controller implemented in [1] finds a close estimate of the unique positive definite solution of the Algebraic Ricatti Equation (ARE) online, in real time, without the need for the knowledge of the internal dynamics matrix of a continuous-time linear system.

### Basic Usage
The IRL algorithm is implemented in `online_linreg.m`. It is validated in `experiments.m`.

The first experiment - `exp = 1` - validates the implementation on the example given in [1]. 
The second experiment - `exp = 2` - validates the implementation on the example given in the Matlab code `ONLINEpolicyiteration_lin.m` found in [2] as [Link to Vrabie software for continuous-time ADP](https://lewisgroup.uta.edu/code/vrabie.zip).

The `online_linreg.m` is defined as a Matlab function that can be used in custom controller implementations.

### TODO
- troubleshoot: response of the controller to changes in dynamics (not working)
- visualization: plots from Vrabie et al. not reproduced yet (validation based on the comparison b/w the P matrix obtained from the ARE and the P matrix obtained from the IRL algorithm)

### References
[1] Vrabie, D., Pastravanu, O., Abu-Khalaf, M., and Lewis, F.L., "Adaptive Optimal Control for Continuous-Time Linear Systems Based on Policy Iteration," Automatica, Vol.  45, pp. 477-484, 2009.

[2] [Software Related to Funded Research of F.L. Lewis](https://lewisgroup.uta.edu/code/Software%20from%20Research.htm)

