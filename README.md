# Static Optimization in Matlab
This code solves the muscle redundancy problem using static optimization in Matlab. Cost and constraing functions can be defined in Matlab using the OpenSim API. Details about the implementation can be found in Uhlrich et al., 2021.

## Publications
Uhlrich SD, Jackson RW, Seth A, Kolesar JA, Delp SL, 2021. Muscle coordination retraining inspired by musculoskeletal simulations: a study on reducing joint loading. bioRxiv. doi: https://doi.org/10.1101/2020.12.30.424841. 

## Running example code
Clone the repository and use the MAIN_StaticOptimization.m script to run the code for example walking data. 

To run this code using your own data, you must scale a model and run Inverse Kinematics and Inverse Dynamics. Then change the paths in MAIN_StaticOptimization.m and settings to match your data and desired simulation settings.

## Functionality
Like the OpenSim static optimization algorithm, this approach solves for muscle activations at each timestep, however there are several differences.
1. This code has the option to estimate muscle lengths with a compliant tendon (see Uhlrich et al. 2020 for details). When tendon compliance is used, the force-velocity multiplier is set to 1.
2. This code has the option to include passive muscle forces.
3. This code allows for the cost and constraint functions to be defined in Matlab. Without changing settings, it minimizes the sum of squared muscle activations, but adding EMG or EMG ratio tracking to the cost or constraint function is also implemented. Changing the weight on individual actuators (muscles or coordinate actuators) is also implemented. Adding quantities that can be computed with a model and state using the OpenSim API (e.g. joint reaction forces) could also be easily added to the cost or constraint functions. For quantities that require changing the state of the model in the cost/constraint function, the 'DynamicsConstraint_momentMatching.m' constraint function must be replaced by the 'DynamicsConstraint_accelerationMatching.m' function in the 'StaticOptimizationAPI_Vectorized.m' function. The "momentMatching" implementation pre-computes all model parameters before the optimization, so does not need to call the OpenSim API when evaluating the cost or constraint functions, increasing computational efficiency. If the model state must be updated to compute the cost or constraint, then the "acceleration matching" technique must be used where the model's joint accelerations are constrained to match the accelerations computed by differentiating the inverse kinematics results. 

## Example data
Data for a 26-year-old healthy male are included in the TestData folder. Inverse Kinematics and Inverse Dynamics have already been run. For more data and examples, view the <a href="https://simtk.org/projects/coordretraining">Coordination Retraining Project </a> on SimTK.

