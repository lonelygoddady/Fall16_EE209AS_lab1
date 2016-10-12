****************Important notice*******************

The lab code will not be functional if the rvcrobot 
toolbox (can be found in robot-9.10.zip folder)
is not preinstalled. The main function uses the Jacobian
function and pesudo-inverse function of the toolbox,
which is much faster than matlab's original functions.
IK and FK functions are not used. 

After rvcrobot is installed, run Robot.m to perform 
IK, FK and modified IK trajectory. Please refer to 
matlab workplace variables to check for detailed 
results. 
 
Please also make sure transfer_m.m is under the same 
folder as Robot.m. It contains the transfer matrix which
is intensivily called by Robot.m.

 
