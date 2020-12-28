function f = CostFunction(coeffs,params) ;
% Runs model simulation for given coefficients, returns integration
%	coeffs 	= initial set of control values
%	params 	= optimization parameters; simulation parameters and 
%			  pointers to instantiated OpenSim objects.


vars4Minimization = [1:params.nMuscles, (params.actuatorsForIDmatching + params.nMuscles)] ; % If actuator is on INPUTS.prescribedActuation Coords list, we don't want to minimize its control

% Compute activation ratios act1/(act1+act2)

% % Compute Cost Function
activComponent = sum(params.weights(vars4Minimization) .* coeffs(vars4Minimization).^2) ;
if params.useEmgRatios

% Can put EMG tracking things in the cost function!
%     actRatios = coeffs(params.coeffRatioInds(:,1))'./(sum(coeffs(params.coeffRatioInds(:,:))));
%     emgRatioComponent = sum(params.weights(params.nActuators+1:params.nActuators+params.nRatios)' .* (actRatios-params.emgRatio_step).^2) ;
    emgRatioComponent = 0 ;

else
    emgRatioComponent = 0 ;
end

f = activComponent + emgRatioComponent ;
% fprintf('activComponent is %.1f and emg component is %.1f \n',activComponent,emgRatioComponent)
    