function [c ceq] = DynamicsConstraint(coeffs,params) ;
% Runs model simulation for given coefficients, returns integration
%	coeffs 	= initial set of control values
%	params 	= optimization parameters; simulation parameters and 
%			  pointers to instantiated OpenSim objects.

% import org.opensim.modeling.*

% % Get a reference to the model and states
osimModel = params.model ;
state = params.state ;
coords = params.coords ;
muscles = params.muscles ;
nActuators = params.nActuators ;
nFreeCoords = params.nFreeCoords ;
nMuscles = params.nMuscles ;
coordVelNames = params.coordVelNames ;
% stateVectorYMuscleInds = params.stateVectorYMuscleInds; 
% stateVectorY = params.stateVectorY ;
controls = params.controls ;
normalMuscInds = params.normalMuscleInds_ML ;

% % % Set Muscle Activations by changing the state vector directly
%This works the same as setActivation when there aren't reserve actuators.
%But it gives a state error when I turn the reserves on. It was only
%marginally (10%) faster using the arm26 model than setActivation.
% for i = 1:nMuscles
%     stateVectorY.set(stateVectorYMuscleInds(i),coeffs(i))
% end
% state.setY(stateVectorY) ;
% osimModel.calcMassCenterVelocity(state) ; % This does realizeDynamics to re initialize position and velocity info for the state


% Set coordinate actuator controls
%     Assumes you have all coordinates actuated with a coordinate actuator.
%     Also assumes coeffs matrix has all muscles first, then coord
%     actuators
for i = muscles.getSize:nActuators-1
    controls.set(i, coeffs(i+1)) ;
end
osimModel.setControls(state,controls) ;

% % Special Case Activation Equality Constraints
nRatios = size(params.coeffRatioInds,1) ;
nEquals = size(params.equalMuscleInds_ML,1) ;
actConstraints = zeros(1,nRatios+nEquals) ;

% % Special Case: Sets muscle activation ratios
if params.useEmgRatios
        ineqDif = 0.02 ; % allow this much difference on either side
        actRatios = coeffs(params.coeffRatioInds(:,1))'./(sum(coeffs(params.coeffRatioInds(:,:))));
        emgWeights = params.weights(nActuators+1:end) ;
        actConstraints(1:nRatios) = emgWeights.* (abs(actRatios-params.emgRatio_step)-ineqDif) ;
end

% % Special Case: Sets muscle activations of some muscles equal to each
% other with some wiggle room
if params.useEqualMuscles
      ineqDif = 0.02 ; % allow this much difference on either side
      actConstraints(nRatios+1:nRatios+nEquals) = ...
          abs((coeffs(params.equalMuscleInds_ML(:,2)) - ...
          coeffs(params.equalMuscleInds_ML(:,1)))') - ineqDif ;
end

% % % Set Muscle Activations the slow way
for i = 1:nMuscles% (IMPROVE ME) this is really slow - it takes ~75% of the whole simulation time
    muscles.get(i-1).setActivation(state,coeffs(i)) ;
end

% % Equillibrate Muscles in Model
osimModel.equilibrateMuscles(state);
% % Get qdd_sim from model - only for the coordinate speeds
osimModel.computeStateVariableDerivatives(state) ; % Osim3.3 gave back a vector here - no such luck in 4.0.

qdd_sim = zeros(size(params.qddIK_step)) ;
for i = 0:nFreeCoords-1 
%     qdd_sim(i+1) = stateDerivs.get(params.coordVelIndicies_ML(i+1)-1) ; 
    qdd_sim(i+1) = osimModel.getStateVariableDerivativeValue(state,coordVelNames{i+1}) ;
end
    
% %Compute differences in accelerations
ceq= [qdd_sim-params.qddIK_step] ;
c = actConstraints ;

