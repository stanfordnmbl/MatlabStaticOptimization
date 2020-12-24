function OUT = getMuscleParams(params,coeffs_initial) ;
% Pre-computes the following variables and stores in structure.
% Note: state must be realized through velocity!
%
% passiveForce = 1 x nMuscles vector of passive muscle forces
% activeForceMult = 1 x nMuscles vector containing sum of F_0_m*f_act(l)*f_act(v)
% cosAlpha = 1 x nMuscles of cosine of pennation angle
% momentArms = nMuscles x nFreeCoords of moment arms
% coordOptForce = 1 x nFreeCoords of optimal force for coordinate actuators


import org.opensim.modeling.*

% Initialize Matricies
OUT.passiveForce = zeros(1,params.nMuscles) ;
OUT.activeForceMult = OUT.passiveForce ;
OUT.cosAlpha = OUT.passiveForce ;
OUT.momentArms = zeros(params.nMuscles,params.nFreeCoords) ;
OUT.coordOptForce = zeros(1,params.nFreeCoords) ;

if ~params.ignoreTendonCompliance
%     try % this is a hack for thelen muscle that doesn't equilibrate sometimes...
        for i = 0:params.nMuscles-1
            params.muscles.get(i).setActivation(params.state,coeffs_initial(i+1)) ;
        end
        params.model.equilibrateMuscles(params.state) ;
%     catch
%         warning('muscles didnt equilibrate - reduced activation 10% to try again')
%         for i = 0:params.nMuscles-1
%             params.muscles.get(i).setActivation(params.state,0.9*coeffs_initial(i+1)) ;
%         end
%         params.model.equilibrateMuscles(params.state) ;
%     end
    
    
else % rigid tendon, still equilibrate Muscles (may be unneccesary)
    params.model.equilibrateMuscles(params.state) ;
end

for i = 1:params.nMuscles
    OUT.passiveForce(i) = max([params.muscles.get(i-1).getPassiveFiberForce(params.state),0]) ;
    OUT.activeForceMult(i) = params.muscles.get(i-1).getActiveForceLengthMultiplier(params.state)* ...
                             params.muscles.get(i-1).getForceVelocityMultiplier(params.state)* ...
                             params.muscles.get(i-1).getMaxIsometricForce ;
    OUT.cosAlpha(i) = params.muscles.get(i-1).getCosPennationAngle(params.state) ;
    for j = 1:params.nFreeCoords
        OUT.momentArms(i,j) = params.muscles.get(i-1).computeMomentArm(params.state,params.coords.get(params.freeCoordsNames{j})) ;
    end
end

for i = 1:params.nFreeCoords
    thisCoordActuator = CoordinateActuator.safeDownCast(params.actuators.get(params.nMuscles-1+i)) ;
    OUT.coordOptForce(i) = thisCoordActuator.get_optimal_force ;
end

    
                  

