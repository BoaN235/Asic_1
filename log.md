# IK Calculator Chip
### Terms
- IK (Inverse Kinematics) - Mathmatical proccess that calculate required joint angles of a kinematic chain. Used in Robotics and animations.
- Verilog - A programing langauge kinda like C that has rules that allow it to be complied through tools directly to logic gates. There is more modern ones however this is the most common one.
: 
### Why?
I just kinda yap I will fix it later

Some of the reaserch I have seen has shown that this can be a useful way to help improve the latency between command and action in surigcal robots. (Source: [ADS Harvard](https://ui.adsabs.harvard.edu/abs/2020aris.conf...25T/abstract))
On top of that robots need to handle more and more heavy programs so by offloading many of those compuational exspensve task such as calculate angles would make them much faster. While these small gains in computation time may not be usefull now this also helps bring down power consupion if the actual board was manufactered. We are also reaching limits in size of logic gates the next step is how we place them. Imagine a computer with specifc cores for mathmatical operations. Allowing for a super fast proccsesing time of complicated tasks such as rendering, complieng and IK.

### TOTAL TODO
1. Learn Verilog with HDLbites
2. Figure out IK

## Oct 13th Notes
### Today's TODO:
1. Learn Verilog through HDLbites 
2. Figure out how IK works

### Analytical vs. Numerical

Analytical IK: is based on mathmatical formulas and closed loops. This would make the most sense within a computer chip

Numerical IK: uses iterative aproaches to find a angles in complex or constrained chains. This would could use a seperate procceser that handles the feeding of information to just get the math done fast.

### Types of IK

#### 1 


# THE PLAN
Build a IK Co - Proccesor

