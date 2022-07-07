# Static Optimization in Matlab
This code solves the muscle redundancy problem using static optimization in Matlab. Cost and constraing functions can be defined in Matlab using the OpenSim API. Details about the implementation can be found in Uhlrich et al., 2021.

## Publications
Please cite this paper if you use this code in your work:
<br>
Uhlrich SD, Jackson RW, Seth A, Kolesar JA, Delp SL, 2022. Muscle coordination retraining inspired by musculoskeletal simulations reduces knee contact force. *Scientific Reports* __12__, 9842. https://doi.org/10.1038/s41598-022-13386-9.  

## Running demo
Install the latest version of OpenSim (this code has been tested with OpenSim 4.2 and Matlab R2020b), and follow the <a href="https://simtk-confluence.stanford.edu/display/OpenSim/Scripting+with+Matlab">instructoins </a> to set up OpenSim scripting in Matlab.

Running the demo requires MATLAB's Optimization Toolbox and the DPS System or Signal Processing Toolbox. Information about acquiring MATLAB toolboxes can be found on the [MathWorks website](https://www.mathworks.com/products/alphabetical.html). 

Clone the repository and use the MAIN_StaticOptimization.m script to run the code to solve for muscle activations for the provided example stance phase of walking. This should take around 50s on a normal desktop computer and produce an interactive plot of muscle activations and reserve actuator controls, shown in Figure 1.

![alt text](https://github.com/stanfordnmbl/MatlabStaticOptimization/blob/main/TestData/activationExampleOutput.jpg)
Figure 1: Example interactive Matlab figure showing the resulting muscle activations and actuator controls from a stance phase of walking.

## Example data
Data for a 26-year-old healthy male walking on a treadmill are included in the TestData folder. Inverse Kinematics and Inverse Dynamics have already been run. For more data and examples, view the <a href="https://simtk.org/projects/coordretraining">Coordination Retraining Project </a> on SimTK.

## Running static optimization with your own data
To run this code using your own data, you must scale a model and run Inverse Kinematics and Inverse Dynamics. Then change the paths in MAIN_StaticOptimization.m and settings to match your data and desired simulation settings.

## Functionality
Like the OpenSim static optimization algorithm, this approach solves for muscle activations at each timestep, however there are several differences.
1. This code has the option to estimate muscle lengths with a compliant tendon (see Uhlrich et al. 2020 for details). When tendon compliance is used, the force-velocity multiplier is set to 1.
2. This code has the option to include passive muscle forces.
3. This code allows for the cost and constraint functions to be defined in Matlab. Without changing settings, it minimizes the sum of squared muscle activations, but adding EMG or EMG ratio tracking to the cost or constraint function is also implemented. Changing the weight on individual actuators (muscles or coordinate actuators) is also implemented. Adding quantities that can be computed with a model and state using the OpenSim API (e.g. joint reaction forces) could also be easily added to the cost or constraint functions. For quantities that require changing the state of the model in the cost/constraint function, the 'DynamicsConstraint_momentMatching.m' constraint function must be replaced by the 'DynamicsConstraint_accelerationMatching.m' function in the 'StaticOptimizationAPI_Vectorized.m' function. The "momentMatching" implementation pre-computes all model parameters before the optimization, so does not need to call the OpenSim API when evaluating the cost or constraint functions, increasing computational efficiency. If the model state must be updated to compute the cost or constraint, then the "acceleration matching" technique must be used where the model's joint accelerations are constrained to match the accelerations computed by differentiating the inverse kinematics results. 
