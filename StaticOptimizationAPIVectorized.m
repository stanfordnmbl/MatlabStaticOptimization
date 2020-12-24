function [] = StaticOptimizationAPI(INPUTS) ;
% This function performs static optimization through the OpenSim API. 

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
import org.opensim.modeling.*

% % % % % THESE ARE USER INPUTS FROM INPUTS STRUCTURE FROM MAIN LOOP
trialname = INPUTS.trialname ;
leg = INPUTS.leg ; 

modelDir = INPUTS.modelDir ;
modelName = INPUTS.modelName ;
ikFilePath = INPUTS.ikFilePath ;
idFilePath = INPUTS.idFilePath ;
outputFilePath = INPUTS.outputFilePath ;

filtFreq = INPUTS.filtFreq; % Lowpass filter frequency for IK coordinates. -1 if no filtering
startTime = INPUTS.startTime ; % seconds, 0 for beginning 
endTime = INPUTS.endTime ; % seconds, large number for end eg. 100000

% Flags
appendActuators = INPUTS.appendActuators ; % Append reserve actuators at all coordinates?
appendForces = INPUTS.appendForces ; % Append grfs?
ignoreTendonCompliance = INPUTS.ignoreTendonCompliance ; % true to make everything have a stiff tendon
deleteContralateralMuscles = INPUTS.deleteContralateralMuscles ; % replace muscles on contralateral leg with powerful reserve actuators
useEmgRatios = INPUTS.useEmgRatios ; % 'false if removing this part from the cost/constraint function
useEqualMuscles = INPUTS.useEqualMuscles ; % 'false if removing this part from the cost/constraint function
useEmgConstraints = INPUTS.useEmgConstraints ; % false if you don't want to constrain muscle activations to input EMG
changePassiveForce = INPUTS.changePassiveForce ; % true if messing with passive force curves

% DOF to not match ID moments 
fixedDOFs = INPUTS.fixedDOFs ;

% EMG file
emgFilePath = INPUTS.emgFilePath ; % location of *.mot file with normalized EMG
emgRatioPairs = INPUTS.emgRatioPairs ; % nPairs x 2 cell
equalMuscles = INPUTS.equalMuscles ; % nPairs x 2 cell of muscles for whom you want equal activations
emgConstrainedMuscles = INPUTS.emgConstrainedMuscles ; % nMuscles x 1 cell of muscles for which you want activation to track EMG
emgSumThreshold = INPUTS.emgSumThreshold ; % If sum of emg pairs is less than this, weight this ratio to 0 in cost function
% * * * THIS DOESN'T DO ANYTHING ANYMORE NOW THAT THE EMG IS IN THE CONSTRAINT

% Weights for reserves, muscles, emgRatios. If you want this on an individ
% basis, will need to implement some other system. This gives the same
% thing to everything in the class (except for emgRatios which are
% individually tunable b/c they are defined below). You can always tune the
% optimal force for the reserves as well if you'd like. The weight is in
% the cost function as sum(w*(whatever^2)), so the weight is not squared.
reserveActuatorWeights = INPUTS.reserveActuatorWeights ; 
muscleWeights = INPUTS.muscleWeights ;
weightsToOverride = INPUTS.weightsToOverride ;
overrideWeights = INPUTS.overrideWeights ;
ipsilateralActuatorStrength = INPUTS.ipsilateralActuatorStrength ;
contralateralActuatorStrength = INPUTS.contralateralActuatorStrength ;
emgRatioWeights = repmat([1],1,size(emgRatioPairs,1)) ; % 1 x npairs weighting of EMG ratio in cost function - this should stay 1 as it is now in the constraints
prescribedActuationCoords = INPUTS.prescribedActuationCoords ;

% External Forces Definitions
forceFilePath = INPUTS.forceFilePath ;  % Full path of forces file

externalForceName = INPUTS.externalForceName ; % nForces x 1 cell
applied_to_body = INPUTS.applied_to_body ; 
force_expressed_in_body = INPUTS.force_expressed_in_body ;
force_identifier = INPUTS.force_identifier ;
point_expressed_in_body = INPUTS.point_expressed_in_body ;
point_identifier = INPUTS.point_identifier ;

% Joint Reaction Fields
jRxn.inFrame = INPUTS.jRxn.inFrame ;
jRxn.onBody = INPUTS.jRxn.onBody ;
jRxn.jointNames = INPUTS.jRxn.jointNames ;
% % % % % % % % % % %

passiveForceStrains = INPUTS.passiveForceStrains ;

% % % % % % END OF INPUTS % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %

% Create output directory and delete prior log
try ; warning off ; mkdir(outputFilePath) ; delete([outputFilePath 'staticOptAPI_log.txt']) ; warning on; end

diary([outputFilePath 'staticOptAPI_log.txt']) ;
fprintf('\n\nBeginning Optimization for %s from %.2f to %.2f seconds. \n\n',trialname,startTime,endTime) ;

tic
if leg == 'r' ; contraLeg = 'l' ; else ; contraLeg = 'r' ; end

% Open model
modelFile = strcat(modelDir,modelName);
osimModel = Model(modelFile);

% Get coordinates
coords = osimModel.getCoordinateSet();
nCoords = coords.getSize() ;

% Get force set
forceSet = osimModel.getForceSet() ;
nForces = forceSet.getSize ;
            
% Find free coordinates 
% anything that isn't the knee angle beta (this is specific to
% Rajagopal2015. A more general way of finding constrained coordinates is
% needed. Also anything that is a joint with ground is supported with a
% coordinate actuator, so we don't need to put that in the optimization.
    freeCoordsNames = {} ; 
    for i = 0:nCoords-1
        if isempty(strfind(char(coords.get(i).getName),'beta')) && coords.get(i).get_locked == 0 ... 
                && coords.get(i).get_prescribed == 0 ... 
                && isempty(strmatch(char(coords.get(i).getName),fixedDOFs,'exact')) ...
            freeCoordsNames{end+1} = char(coords.get(i).getName) ;
        end
    end
    nFreeCoords = length(freeCoordsNames) ;
    
% Append reserve actuators
if appendActuators
    %Delete lumbar coord actuators
    forceNamesToDelete = {}; 
    for i = 1:nForces
        if ~isempty(strfind(char(forceSet.get(i-1).getName()),['lumbar']))
             forceNamesToDelete{end+1} = char(forceSet.get(i-1).getName) ;
        end
    end
    for i = 1:length(forceNamesToDelete)
        forceSet.remove(forceSet.get(forceNamesToDelete{i})) ;
    end
    nForces = forceSet.getSize ;
    
    % Get force names
    forceNames = cell(nForces,1) ;
    for i = 1:nForces
       forceNames{i} = char(forceSet.get(i-1).getName()) ;
       if strfind(forceNames{i},'_reserve') ;
           disp([forceNames{i} ' has already been appended. Please delete. Program will append it anyway.'])
       end
    end
    
    for i = 0:nCoords-1
     if ~isempty(strmatch(char(coords.get(i).getName),freeCoordsNames)) || ~isempty(strmatch(char(coords.get(i).getName),prescribedActuationCoords))  ; % only append actuator if freeCoord
%         disp(['adding ' char(coords.get(i).getName) '_reserve to actuator set'])
         newActuator = CoordinateActuator(char(coords.get(i).getName)) ;
        newActuator.setName([char(coords.get(i).getName()) '_reserve'])
%         newActuator.set_coordinate(coords.get(i).getName())
        newActuator.set_min_control(-Inf)
        newActuator.set_max_control(Inf)
        coordinateName = char(coords.get(i).getName) ;
        if ~isempty(strfind(coordinateName(end-1:end),['_' contraLeg])) && deleteContralateralMuscles % coordinate actuators for contralateral leg
            newActuator.set_optimal_force(contralateralActuatorStrength) % make it cheap to actuate other leg
        elseif ~isempty(strfind(coordinateName,'pelvis')) || ~isempty(strfind(coordinateName,'lumbar')) 
            newActuator.set_optimal_force(contralateralActuatorStrength) % Cheap to make it dynamically consistent
        elseif ~isempty(strmatch(coordinateName,prescribedActuationCoords))
            newActuator.set_optimal_force(contralateralActuatorStrength) % Cheap to make it dynamically consistent
        else
            newActuator.set_optimal_force(ipsilateralActuatorStrength)
        end    
        osimModel.addForce(newActuator);
     
     % Add Prescribed Controllers for Coordinate Actuators (Needed for joint
     % reaction force to work properly)
     
     % Construct piecewise constant function
     constFxn = Constant(0) ;
     constFxn.setName([char(coords.get(i).getName) '_constFxn']) ;
     
     % Construct prescribed controller
     pController = PrescribedController() ;
     pController.setName([char(coords.get(i).getName) '_controller']) ;
     pController.addActuator(newActuator) ;
     pController.prescribeControlForActuator(0,constFxn) ; % attach the function to the controller
     osimModel.addController(pController) ;
     end % freeCoords
    end
end %appendActuators

controllerSet = osimModel.getControllerSet ;

% Get Actuators
actuators = forceSet.updActuators ;
nActuators = actuators.getSize ;

nForces = forceSet.getSize() ;

% Get muscles
muscles = osimModel.updMuscles() ;
nMuscles = muscles.getSize() ;
   % Get muscle Names
   muscleNames = cell(1,nMuscles) ; ;
   for i = 0:nMuscles-1
       muscleNames{i+1} = char(muscles.get(i).getName) ;
   end
   
   % Ignore tendon compliance
   if ignoreTendonCompliance 
      for i = 0:nMuscles-1
         muscles.get(i).set_ignore_tendon_compliance(1)
      end
   end
   
   % Change Passive Forces
   if changePassiveForce
       curve = FiberForceLengthCurve(passiveForceStrains(1),passiveForceStrains(2),0.2,2.86,0.75) ;
       for i = 0:nMuscles-1
           musc = Millard2012EquilibriumMuscle.safeDownCast(muscles.get(i)) ;
           musc.set_FiberForceLengthCurve(curve) ;
       end
   end
   
   % Delete Contralateral Leg Muscles
   if deleteContralateralMuscles
       for i = 1:nMuscles
           if ~isempty(strfind(muscleNames{i},['_' contraLeg]))
              forceSet.remove(muscles.get(muscleNames{i})) ;
           end
       end
   end
   
   
% Edit model for external forces 
% Issue Ajay helped solve: Can't identify external data source, but can
% make storage programatically. Bug in OpenSim3.3. This model wouldn't open
% in the OpenSim 3.3 GUI - may work in 4.0.

 if appendForces
    dataSource = Storage(forceFilePath) ;
    for i = 1:length(applied_to_body) ; % nExternalForces
        newForce = ExternalForce() ;
        newForce.setName(externalForceName{i}) ;
        newForce.set_applied_to_body(applied_to_body{i}) ;
        newForce.set_force_expressed_in_body(force_expressed_in_body{i}) ;
        newForce.set_force_identifier(force_identifier{i}) ;
        newForce.set_point_expressed_in_body(point_expressed_in_body{i}) ;
        newForce.set_point_identifier(point_identifier{i}) ;
        newForce.setDataSource(dataSource) ;
        osimModel.addForce(newForce) ;
    end
%     osimModel.print([outputFilePath modelName(1:end-5) '_withGRFS_WontOpenInGUI.osim']);
 end

% % % % NO MORE EDITING THE MODEL AFTER THIS POINT % % % % %

% Get model state
state = osimModel.initSystem ;
nForces = forceSet.getSize ;
nActuators = actuators.getSize ;
nMuscles = muscles.getSize;
 
% Get names of states
stateNames = osimModel.getStateVariableNames ;
nStates = getSize(stateNames) ;
stateNames_ML = cell(nStates,1) ;
for i = 0:nStates-1
    stateNames_ML{i+1} = char(stateNames.get(i)) ;
end

% Get force names
forceNames = cell(nForces,1) ;
for i = 1:nForces
   forceNames{i} = char(forceSet.get(i-1).getName()) ;
   if strfind(forceNames{i},'_reserve') ;
       controlActuatorInds_ML = i ;
   end
end

% Get weight names
actuatorNames = cell(actuators.getSize,1) ;
for i = 1:actuators.getSize
   actuatorNames{i} = char(actuators.get(i-1).getName()) ;
end

% Define weights vector
weights = [repmat(muscleWeights,1,nMuscles), ...
           repmat(reserveActuatorWeights,1,nFreeCoords), ...
           emgRatioWeights]' ; %used as a column vector below

% Set weight vector overrides
if ~isempty(overrideWeights)
    for i = 1:length(overrideWeights)
        weightInd = strmatch(weightsToOverride{i},actuatorNames) ;
        for j = 1:length(weightInd) ;
            weights(weightInd(j)) = overrideWeights(i) ;
        end
    end
end
       
% Create vector of indicies referring to coordinate velocities in state vec
% This is every coord that isn't fixed or constrained. Dynamics constraint
% (non-vectorized version of the code) will match these accelerations

% Get rid of stuff in front of state name (this is a hack to transition to
% Osim4.0 state name definitions)

for i = 1:length(stateNames_ML)
    indsSlash = strfind(stateNames_ML{i},'/');
    stateNames_ML2{i,1} = stateNames_ML{i}(indsSlash(end-1)+1:end) ;
end


coordVelIndicies_ML = zeros(nFreeCoords,1) ;
coordVelNames = cell(nFreeCoords,1) ;
for i = 0:nFreeCoords-1
    coordVelIndicies_ML(i+1) = strmatch(strcat(freeCoordsNames{i+1},'/speed'),stateNames_ML2) ;
    coordVelNames{i+1} = stateNames_ML{coordVelIndicies_ML(i+1)} ;
end


% Get coordinates storage from IK file
coordinateSto=Storage(ikFilePath);
% Get the timestamps and Coordinate values
timeIK=ArrayDouble();
coordinateSto.getTimeColumn(timeIK);
timeIK_ML = str2num(timeIK) ;

% Matlab Array of IK coordinate Values
coordNamesIK = cell(1,nCoords) ;
dataIK = zeros(length(timeIK_ML),nCoords) ;
freeCoordsIndsInIkMatrix_ML = zeros(1,nFreeCoords) ;
counter_freeCoords = 1 ;
for i = 0:nCoords-1
    dcol = ArrayDouble() ;
    coordinateSto.getDataColumn(coords.get(i).getName(),dcol) ;
    dataIK(:,i+1) = str2num(dcol) ;
    coordNamesIK{i+1} = char(coords.get(i).getName()) ;
    % Turn to radians if rotational dofs in degrees
    if coordinateSto.isInDegrees()
        if strcmp(coords.get(i).getMotionType(),'Rotational')
            dataIK(:,i+1) = deg2rad(dataIK(:,i+1)) ; 
        end
    end
    % Find indicies of ik matrix that correspond to freeCoords
    if ~isempty(strmatch(char(coords.get(i).getName()),freeCoordsNames)) ;
        freeCoordsIndsInIkMatrix_ML(counter_freeCoords) = i+1 ; % indexed in Matlab
        counter_freeCoords = counter_freeCoords + 1 ;
    end
end

% Filter Coords with 4th order, zero-lag, butterworth lowpass
sampFreq = 1/(timeIK_ML(2)-timeIK_ML(1)) ; % IK coordinate sample rate
if filtFreq >0
    [b,a] = butter(2,filtFreq/(sampFreq/2)) ; % 4th order butterworth (filtfilt doubles order)
    qIK = filtfilt(b,a,dataIK) ;
else % no filtering
    qIK = dataIK ;
end

% Compute qd_IK and qdd_IK
qdIK = zeros(size(qIK)) ;
qddIK = qdIK ;
for i = 1:nCoords
    qdIK(:,i) = gradient(qIK(:,i),1/sampFreq) ; % gradient uses central difference on interior points, single side difference on edges
    qddIK(:,i) = gradient(qdIK(:,i),1/sampFreq) ;
end

% % Get Inverse Dynamics Moments from File
% Get coordinates storage from IK file
idSto=Storage(idFilePath);
idLabels = idSto.getColumnLabels();
% Get the Time stamps and Coordinate values
timeID=ArrayDouble();
idSto.getTimeColumn(timeID);
timeID_ML = str2num(timeID) ;

for i = 1:idLabels.getSize -1 % this skips the first one which is time
    idLabelsML{i} = char(idLabels.get(i));
end

% Matlab Array of ID free coordinate Values
coordNamesID = cell(1,nFreeCoords) ;
momentsID = zeros(length(timeID_ML),nFreeCoords) ;
for i = 1:length(freeCoordsNames)
    dcol = ArrayDouble() ;
    if strcmp(coords.get(freeCoordsNames{i}).getMotionType(),'Rotational')
        forceType = '_moment' ;
    else
        forceType = '_force' ;
    end
    idColIdx = strmatch([freeCoordsNames{i} forceType],idLabelsML) -1 ;
    
    idSto.getDataColumn(idColIdx,dcol) ;
    momentsID(:,i) = str2num(dcol) ;
    coordNamesID{i} = freeCoordsNames{i} ;
end

% Matlab Array of ID prescribed coordinate Values
if length(prescribedActuationCoords) > 0
    momentsID_prescribed = zeros(length(timeID_ML),length(prescribedActuationCoords)) ;
    actuatorStrength_prescribed = zeros(zeros(1,length(prescribedActuationCoords))) ;
    actuatorPrescribed_inds_ML = actuatorStrength_prescribed ;
    for i = 1:length(prescribedActuationCoords)
        dcol = ArrayDouble() ;
        if strcmp(coords.get(freeCoordsNames{i}).getMotionType(),'Rotational')
            forceType = '_moment' ;
        else
            forceType = '_force' ;
        end
        idColIdx = strmatch([freeCoordsNames{i} forceType],idLabelsML) -1 ;

        idSto.getDataColumn(idColIdx,dcol) ;
        momentsID_prescribed(:,i) = str2num(dcol) ;

        % Get actuator strength
        thisActuator = CoordinateActuator.safeDownCast(actuators.get(char([prescribedActuationCoords{i,1} '_reserve']))) ;
        actuatorStrength_prescribed(i) = thisActuator.getOptimalForce ;
        actuatorPrescribed_inds_ML(i) = strmatch([prescribedActuationCoords{i}],freeCoordsNames) ;
    end
else
    actuatorStrength_prescribed = [] ; % placeholder
    actuatorPrescribed_inds_ML = [] ;
end
actuatorsForIDmatching = find(xor(ones(1,nFreeCoords), ismember(1:nFreeCoords,actuatorPrescribed_inds_ML))) ; % These are the moments that the optimizer will try to match

% % Compute EMG ratio parameters
    coeffRatioInds = [] ;
    nRatios = 0 ;
    
if useEmgRatios || useEmgConstraints || useEqualMuscles
    % Get emg storage from file
    emgSto=Storage(emgFilePath);
    % Get the Time stamps and Coordinate values
    timeEMG=ArrayDouble();
    emgSto.getTimeColumn(timeEMG);
    timeEMG_ML = str2num(timeEMG) ;

    % Matlab Array of EMG Values
    emgColLabels = emgSto.getColumnLabels ;
    nEMGNames = size(emgColLabels)-1 ;
    emgNames = cell(1,nEMGNames) ;
    dataEMG = zeros(length(timeEMG_ML),nRatios*2) ;
    for i = 1:nEMGNames
        emgNames{i} = char(emgColLabels.get(i-1)) ;
    end
end
    
if useEmgRatios
    emgRatioPairs = cellfun(@(x) [x '_' leg],emgRatioPairs,'UniformOutput',false) ;
    % Load EMG *.sto file with EMG for activation ratios
    nRatios = size(emgRatioPairs,1) ;
    
    for i = 1:nRatios*2
        dcol = ArrayDouble() ;
        emgName = [emgRatioPairs{ceil(i/2),2-rem(i,2)} '_activation'] ;
        if ~strmatch(emgName,emgNames)
            error([emgName ' is not in the EMG.sto file.']) ;
        end
        
        %dataEMG will have the consecutive rows of emgRatioPairs 
        emgSto.getDataColumn(emgName,dcol) ;
        dataEMG(:,i) = str2num(dcol) ;
    end
    
    dataEMG = interp1(timeEMG_ML,dataEMG,timeIK_ML) ; % Timesync EMG with IK results
    
    emgRatio = zeros(size(dataEMG,1),nRatios) ;
    emgSum = zeros(size(dataEMG,1),nRatios) ;
    emgRatioWeightCoeff = zeros(size(dataEMG,1),nRatios); 
    for i = 1:nRatios
        emgSum(:,i) = sum(dataEMG(:,1+(i-1)*2:i*2),2) ;
        emgRatio(:,i) = dataEMG(:,1+(i-1)*2)./(emgSum(:,i)); % EMG1/(EMG2+EMG1) for optimization scaling
        emgRatioWeightCoeff(:,i) = double(emgSum(:,i)>emgSumThreshold) ; % binary coefficient for activation ratio weightings. Unweight this term in cost fxn if muscles are below threshold
%         disp('Remember, you messed with your EMG cost function by adding epsilon to denom')
    end

    % Compute indicies of coefficient matrix of muscles for ratios
    coeffRatioInds = zeros(nRatios,1) ;
    for i = 1:nRatios
        coeffRatioInds(i,1) = strmatch(emgRatioPairs{i,1},forceNames) ;
        coeffRatioInds(i,2) = strmatch(emgRatioPairs{i,2},forceNames) ;
    end
end %useEmgRatios

% Find indices of muscles that we will set to the same activation level
equalMuscleInds_ML = zeros(size(equalMuscles,1),2) ;
if length(equalMuscles) >1  
    equalMusclesLeg = cellfun(@(x) [x '_' leg],equalMuscles,'UniformOutput',false) ;
    for i = 1:size(equalMusclesLeg,1)
        equalMuscleInds_ML(i,1) = strmatch(equalMusclesLeg{i,1},forceNames) ;
        equalMuscleInds_ML(i,2) = strmatch(equalMusclesLeg{i,2},forceNames) ;          
    end
else
    equalMuscleInds_ML = [] ;
end

% Find indices of muscles that we will track EMG directly
constrainedMuscleInds_ML = zeros(length(emgConstrainedMuscles),1) ;
if useEmgConstraints
    if size(emgConstrainedMuscles,2)>1 ; error('emgConstrainedMuscles needs to be a column cell, not a row') ; end
    constrainedMuscles = cellfun(@(x) [x '_' leg],emgConstrainedMuscles,'UniformOutput',false) ;
    for i = 1:length(emgConstrainedMuscles)
        constrainedMuscleInds_ML(i,1) = strmatch(constrainedMuscles{i,1},forceNames) ;
    end
    
    emgConstrainedMuscles_leg = cellfun(@(x) [x '_' leg],emgConstrainedMuscles,'UniformOutput',false) ;
    % Load EMG *.sto file with EMG for activation ratios
    nMuscConstraints = length(emgConstrainedMuscles) ;
    
    
    dataEMG = [] ;
    for i = 1:nMuscConstraints
        dcol = ArrayDouble() ;
        emgName = [emgConstrainedMuscles_leg{i} '_activation'] ;
        if ~strmatch(emgName,emgNames)
            error([emgName ' is not in the EMG.sto file.']) ;
        end
        
        %dataEMG will have the consecutive rows of emgRatioPairs 
        emgSto.getDataColumn(emgName,dcol) ;
        dataEMG(:,i) = str2num(dcol) ;
    end
    
    emgConstraintValues = reshape(max(0,interp1(timeEMG_ML,dataEMG,timeIK_ML)),size(dataEMG)) ; % Timesync EMG with IK results
    
    % Compute indicies of coefficient matrix of muscles for ratios
    coeffEmgConstraintInds = zeros(nMuscConstraints,1) ;
    for i = 1:nMuscConstraints
        coeffEmgConstraintInds(i,1) = strmatch(emgConstrainedMuscles_leg{i,1},forceNames) ;
    end
else
    coeffEmgConstraintInds = [] ;
     emgConstraintValues = [] ;
end %useEmgConstraints

% Find indices of muscles that we will get set normally
muscleIndVec = 1:muscles.getSize ;
normalMuscleInds_ML = muscleIndVec(~ismember(muscleIndVec,[reshape(equalMuscleInds_ML,1,[]),reshape(coeffEmgConstraintInds,1,[])])) ;


% % % % Setup Analyses
% ForceReporter
forceReport = ForceReporter(osimModel) ;
forceReport.includeConstraintForces(1); 
osimModel.addAnalysis(forceReport) ;

% StateReporter
stateReport = StatesReporter(osimModel) ;
stateReport.setInDegrees(true) ;
osimModel.addAnalysis(stateReport) ;

% JointReaction
inFrame = ArrayStr ;
onBody = ArrayStr ;
jointNames = ArrayStr ;
inFrame.set(0,jRxn.inFrame) ;
onBody.set(0,jRxn.onBody) ;
jointNames.set(0,jRxn.jointNames) ;

jointRxn = JointReaction() ;
jointRxn.setName('JointRxn') ;
jointRxn.setInFrame(inFrame) ;
jointRxn.setOnBody(onBody) ;
jointRxn.setJointNames(jointNames) ;
jointRxn.setStartTime(1);
osimModel.addAnalysis(jointRxn) ;
jointRxn.setModel(osimModel) ;
jointRxn.print([outputFilePath 'JrxnSetup.xml']) ;


% Find start and end rows
[~, startRow_ML] = min(abs(timeIK_ML-startTime)) ;
[~, endRow_ML] = min(abs(endTime-timeIK_ML)) ;
nTimeSteps = endRow_ML-startRow_ML + 1 ; % Number of iterations through the time loop

% Set up optimizer parameters that don't change with each timestep
    % Set initial coefficients. Muscle activations, then reserve actuator
    % controls, then muscle ratios (if in cost fxn)
    coeffs_initial = 0.125*ones(nActuators,1) ;
    coeffs_lastStep = coeffs_initial ; %initialize activations for first step tendon force estimation
    
    % Set linear equality and inequality constraints
    A = [] ;
    Aeq = [] ;
    b = [] ;
    beq = [] ;
    
    % Set boundary constraints
    % Only dealing with muscles and coordinate actuators here
    lb = zeros(1,nFreeCoords) ;
    ub = lb ;
    for i = 1:nMuscles+nFreeCoords
        if strcmp(forceSet.get(i-1).getConcreteClassName(),'CoordinateActuator')
            lb(i) = -inf ;
            ub(i) = inf ; 
        elseif strfind(forceSet.get(i-1).getConcreteClassName(),'Muscle')
            lb(i) = 0 ;
            ub(i) = 1 ;            
        else
            warning(strcat(char(forceSet.get(i-1).getConcreteClassName()),'is not a muscle or coordinate actuator'))
            lb(i) = -inf ;
            ub(i) = inf ; 
        end
    end
    
    % Set optimizer options
    options_sqp = optimoptions('fmincon','Display','notify-detailed', ...
         'TolCon',1e-4,'TolFun',1e-12,'TolX',1e-8,'MaxFunEvals',20000,...
         'MaxIter',5000,'Algorithm','sqp');
    options_ip = optimoptions('fmincon','Display','notify-detailed', ...
         'TolCon',1e-4,'TolFun',1e-12,'TolX',1e-8,'MaxFunEvals',1000000,...
         'MaxIter',10000,'Algorithm','interior-point');

% Preallocate design variable output matrix
Activations = zeros(nTimeSteps,nActuators) ;
timePerStep = zeros(nTimeSteps) ;
costVal = zeros(nTimeSteps,1) ;
timeVec = zeros(nTimeSteps,1) ;
stateVector_ML = zeros(nStates,1) ;

% Set unchanging variables to pass to optimizer in params
params.coords = coords ;
params.forceSet = forceSet ;
params.actuators = actuators ;
params.nActuators = nActuators ;
params.nMuscles = nMuscles ;
params.nFreeCoords = nFreeCoords ;
params.muscles = muscles ;
params.nForces = nForces ;
params.useEmgRatios = useEmgRatios ;
params.useEqualMuscles = useEqualMuscles ;
params.useEmgConstraints = useEmgConstraints ;
params.nStates = nStates ;
params.coordVelIndicies_ML = coordVelIndicies_ML ;
params.coordVelNames = coordVelNames ;
params.equalMuscleInds_ML = equalMuscleInds_ML ;
params.normalMuscleInds_ML = normalMuscleInds_ML ;
params.coeffRatioInds = coeffRatioInds ;
params.coeffEmgConstraintInds = coeffEmgConstraintInds ;
params.emgConstraintValues = emgConstraintValues ;
params.nRatios = nRatios ;
params.freeCoordsNames = freeCoordsNames ;
params.ignoreTendonCompliance = ignoreTendonCompliance ;
params.actuatorsForIDmatching = actuatorsForIDmatching ;
params.actuatorPrescribed_inds_ML = actuatorPrescribed_inds_ML ;
params.actuatorStrength_prescribed =  actuatorStrength_prescribed ;



tLast = toc ;
% % Time-stepping Optimization Loop
for tInd_ML = 1:nTimeSteps ; % counter is Matlab indexing
    rowInd_ML = startRow_ML-1+tInd_ML ; % Matlab Row Index
    t = timeIK_ML(rowInd_ML) ;
    timeVec(tInd_ML) = t ;
    fprintf('\nOptimizing t=%.3fs\n',t)
    
    state.setTime(t) ; % This clears all positional and velocity information in the state

    % set coordinate qs and q's from IK (all coordinates, even locked and
    % constrained ones
    for i = 1:nCoords
        % Set model state variables (q, qd)
        coords.get(coordNamesIK{i}).setValue(state,qIK(rowInd_ML,i));
        coords.get(coordNamesIK{i}).setSpeedValue(state,qdIK(rowInd_ML,i)) ; 
    end
    osimModel.realizeVelocity(state) ;

    
    % set activation ratio weights to 0 if below emgSumThreshold
    if useEmgRatios
        weightsStep = weights .* [ones(1,nActuators) emgRatioWeightCoeff(rowInd_ML,:)]' ;
        params.emgRatio_step = emgRatio(rowInd_ML,:) ;
        params.emgRatio_sum = emgSum(rowInd_ML,:) ;
    else
        weightsStep = weights ;
    end
    
    if useEmgConstraints
        params.emgConstraints_step = emgConstraintValues(rowInd_ML,:) ;
    end
    
    % Set State Vector Values for each time step (slow, but faster than
    % doing it in dynamics constraint
    for i = 1:nStates
        stateVector_ML(i) = osimModel.getStateVariableValue(state,stateNames.get(i-1)) ;
    end
    
    controlVec = Vector(nActuators,0) ;
    controls = Vector(nActuators,0) ;
    
%     osimModel.calcMassCenterVelocity(state) ; % This does realizeDynamics to re initialize position and velocity info for the state
    
    % Set changing variables to pass to optimizer in params
    params.model = osimModel ;
    params.state = state ;
    params.qddIK_step = qddIK(rowInd_ML,freeCoordsIndsInIkMatrix_ML) ;
    params.weights = weightsStep ;
    params.stateVector_ML = stateVector_ML; 
    params.stateVectorY = state.getY ;
    params.controls = controlVec ;
    params.moments_ID = momentsID(rowInd_ML,:) ;
    
    % Compute fixed muscle parameters (moment arms, active force
    % multipliers, etc.)
    params.muscParams = getMuscleParams(params,coeffs_lastStep) ;
    
    % Set nonlinear constraint function
    nonlcon = @(coeffs0) DynamicsConstraint_momentMatching(coeffs0,params) ;
    
    % Call constrained optimization
    try % Use SQP optimizer first
        [coeffsFinal,fval,exitflag,output] = fmincon(@(coeffs0) CostFunction(coeffs0,params), ...
        coeffs_initial,A,b,Aeq,beq,lb,ub,nonlcon,options_sqp) ;
        if tInd_ML >1 && fval>2*fvalLast % after first iteration, look for spikes in cost function - use IP if identified
            disp(['Cost Value was ' num2str(ceil(fval/fvalLast)) 'x last time, will try to use IP'])
            error('Cost Too High')
        elseif exitflag < 1
            disp(['SQP optimizer had an issue - output state was ' num2str(exitflag)])
            error('Optimizer Failed')
        end
    catch % Use IP if having issues with SQP
            disp('Trying IP instead') ;
            [coeffsFinal,fval,exitflag,output] = fmincon(@(coeffs0) CostFunction(coeffs0,params), ...
            coeffs_initial,A,b,Aeq,beq,lb,ub,nonlcon,options_ip) ;
    end
    fvalLast = fval ;
    
    if useEmgRatios
    actRatios = coeffsFinal(params.coeffRatioInds(:,1))'./(sum(coeffsFinal(params.coeffRatioInds(:,:)))) ;
    emgRatioComponent = sum(params.weights(params.nActuators+1:params.nActuators+params.nRatios)' .* (actRatios-params.emgRatio_step).^2) ;
    activComponent = sum(params.weights(1:params.nActuators) .* coeffsFinal(1:params.nActuators).^2) ;
    
    for i = 1:nRatios
        fprintf(['Activation ratio: %.2f, EMG ratio: %.2f. EMGsum: %.2f, EMGsumThreshold: %.2f. Cost: %.2f.\n'] ...
             ,actRatios(i),params.emgRatio_step(i),emgSum(rowInd_ML,i),emgSumThreshold,activComponent)
    end
    end
    
    % % Prescribe reserve actuator controls so that M_act+M_muscle = M_ID
    % ONLY FOR MUSCLES IN INPUTS.prescribedActuationCoords
    if ~isempty(INPUTS.prescribedActuationCoords)
        MP = params.muscParams ;
        moments_sim_muscles = ((coeffsFinal(1:nMuscles)' .* MP.activeForceMult + MP.passiveForce) .* MP.cosAlpha) * MP.momentArms ;
        moments_sim_actuators = coeffsFinal(nMuscles+1:nMuscles+nFreeCoords)' .* MP.coordOptForce ;
        moments_sim = moments_sim_muscles + moments_sim_actuators ;

        coeffsFinal(nMuscles+params.actuatorPrescribed_inds_ML) = (params.moments_ID(params.actuatorPrescribed_inds_ML)-moments_sim_muscles(params.actuatorPrescribed_inds_ML)) ./ params.actuatorStrength_prescribed ;
    end
    
    % % % Post-hoc Analyses
    % % % % % % % % % % % % 
    % Set the state vector based on the solution for this timestep for
    % force, state, and joint reaction reporters    
        
    % Set prescribed controller constant value to control value. Controls
    % don't live through joint reaction. A new state is declared, so the
    % cache is deleted and this is a workaround.
    for i = 0:controllerSet.getSize-1
         thisController = PrescribedController.safeDownCast(controllerSet.get(i)) ;
         thisConstFxn = Constant.safeDownCast(thisController.get_ControlFunctions(0).get(0)) ;
         thisConstFxn.setValue(coeffsFinal(nMuscles+1+i)) ;
    end 
    
    % Set Controls. This will work for force reporter
    for i = muscles.getSize:nActuators-1
        controls.set(i, coeffsFinal(i+1)) ;
    end
    osimModel.setControls(state,controls) ;

    % Set muscle activations
    for i = 1:nMuscles
        muscles.get(i-1).setActivation(state,coeffsFinal(i))
    end
      
    osimModel.equilibrateMuscles(state);
    osimModel.realizeAcceleration(state) ; % moves stage back to acceleration after setting control function

    % Run Analyses for full static optimization solution
    if tInd_ML == 1
        forceReport.begin(state) ;
        stateReport.begin(state) ;
        jointRxn.begin(state) ;
    else
        forceReport.step(state,tInd_ML-1) ;
        stateReport.step(state,tInd_ML-1) ;
        jointRxn.step(state,tInd_ML-1) ;
    end
      
    
    
    % Store activations
    coeffs_lastStep = coeffsFinal ; % This is also used to compute tendon force, so don't change without checking that
    costVal(tInd_ML) = fval ;
    Activations(tInd_ML,:) = coeffsFinal ;
    timeForThisIteration = toc - tLast ;
    timePerStep(tInd_ML) = timeForThisIteration ;
    tLast = toc ;
    disp(['This step took ' num2str(timeForThisIteration) 's. Exitflag was ' num2str(exitflag) '.']) ;
end % tInd_ML - this is the time loop

% End and Save Analyses to File
warning off
try ; mkdir(outputFilePath) ; end
warning on
% forceReport.end(state);
forceReport.getForceStorage.print([outputFilePath 'results_forces.sto']) ;
stateReport.getStatesStorage.print([outputFilePath 'results_states.sto']) ;
jointRxn.end(state) ;
% jointRxn.printResults([outputFilePath 'results_JointReaction.sto']) ;
jointRxn.printResults('results_JointReaction',outputFilePath,-1,'.sto') ;


% Save Model (clear external Forces first - or will crash Opensim due to
% bug if you try to open in GUI
if appendForces
    for i = 1:length(externalForceName)
        forceSet.remove(forceSet.get(externalForceName{i})) ;
    end
end

state = osimModel.initSystem() ; % Re-build the model before printing
% osimModel.print([modelDir modelName(1:end-5) '_staticOpt.osim']);

% Plot Output Activations
figure
for i = 0:size(Activations,2)-1
    plot(repmat(timeVec,1,size(Activations,2)),Activations,'linewidth',2)
    legNames{i+1} = char(actuators.get(i).getName()) ;
end
xlabel('time')
ylabel('Activation')
legHandle = legend(legNames,'location','northeastoutside') ;
plotbrowser on
toc
diary off