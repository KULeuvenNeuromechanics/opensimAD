% --------------------------------------------------------------------------
% This script uses OpenSimAD to generate a CasADi external function. Given
% an OpenSim model provided as an .osim file, this script generates a C++
% file with a function F building the musculoskeletal model programmatically 
% and running inverse dynamics. The C++ file is then compiled as an .exe, 
% which when run generates the expression graph underlying F. From this
% expression graph, CasADi can generate C code containing the function F
% and its Jacobian in a format understandable by CasADi. This code is 
% finally compiled as a .dll that can be imported when formulating 
% trajectory optimization problems with CasADi.
%
% The function F takes as:
%     - INPUTS: 
%         - joint positions and velocities (intertwined)
%         - joint accelerations
%         - (optional) forces and moments acting on bodies
%     - OUTPUTS:
%         - joint torques
%         - (optional) other variables exported from the model
%
%   See "help generateExternalFunction" for more info
%
% Reference: 
%   Falisse A, Serrancolí G, et al. (2019) Algorithmic differentiation 
%   improves the computational efficiency of OpenSim-based trajectory 
%   optimization of human movement. PLoS ONE 14(10): e0217730. 
%   https://doi.org/10.1371/journal.pone.0217730
%
% Original author: Lars D'Hondt (based on code by Antoine Falisse)
% Original date: 8/May/2023
% --------------------------------------------------------------------------

clear
clc


% Get current directory
pathMain = pwd;
addpath(fullfile(pathMain,'utilities'))

%% User inputs

% Provide path to OpenSim model.
pathOpenSimModel = fullfile(pathMain, 'examples', 'Hamner_modified.osim');

% Path to folder for outputs
outputDir = fullfile(pathMain, 'examples');

% Output file name
outputFilename = 'F_test';

% Compiler
compiler = 'Visual Studio 17 2022';

% Print information to the command window
verbose_mode = false;

% Verify the generated function
verify_ID = true;

% Order of joints and coordinates. There is no need to provide joints or 
% coordinates, they are retrieved from the .osim file.
jointsOrder = [];
coordinatesOrder = [];

% By default, the external function considers forces due to gravity and
% contact elements. You can add more forces and moment to model e.g. a
% pusher, an exoskeleton or to implement a custom contact model.
input3DBodyForces(1).body = 'torso';
input3DBodyForces(1).point_in_body = [-0.1, 0.3, 0];
input3DBodyForces(1).name = 'back_push';
input3DBodyForces(1).reference_frame = 'ground';

input3DBodyMoments(1).body = 'tibia_l';
input3DBodyMoments(1).name = 'exo_shank_l';
input3DBodyMoments(1).reference_frame = 'tibia_l';
input3DBodyMoments(2).body = 'calcn_l';
input3DBodyMoments(2).name = 'exo_foot_l';
input3DBodyMoments(2).reference_frame = 'tibia_l';

% By default, the external function returns the joint torques. However, you
% can also export other variables that you may want to use when formulating
% your problem. Here, we provide a few examples of variables we typically use
% in our problems.

% Export 3D positions and velocities of points w.r.t. ground reference.
% Leave empty to not export those variables.
export3DPositions = [];
export3DPositions(1).body = 'tibia_l';
export3DPositions(1).point_in_body = [0, -0.012, 0];
export3DPositions(1).name = 'left_shin';
export3DPositions(2).body = 'tibia_r';
export3DPositions(2).point_in_body = [0, -0.012, 0];
export3DPositions(2).name = 'right_shin';

export3DVelocities = [];
export3DVelocities(1).body = 'tibia_l';
export3DVelocities(1).point_in_body = [0, -0.012, 0];
export3DVelocities(1).name = 'left_shin';

% Export total GRFs.
% If true, right and left 3D GRFs (in this order) are exported.
exportGRFs = true;

% Export separate GRFs.
% If true, right and left 3D GRFs (in this order) are exported for each
% contact sphere. 
exportSeparateGRFs = true;

% Export GRMs.
% If true, right and left 3D GRMs (in this order) are exported.
exportGRMs = true;

% Export contact sphere vertical deformation power.
% If true, right and left vertical deformation power of all contact spheres 
% are exported. 
exportContactPowers = true;

% secondOrderDerivatives
% The generated library always contains the expression graphs to evaluate
% the Jacobian of the external function. If this input is true, expression
% graphs for evaluating second derivative information are also added.
secondOrderDerivatives = false;

% Do not create an external function (.dll and .lib)
% If you only want to use the serialised function (.casadi), this option cn
% save some time.
noDll = false;

%% Call generateExternalFunction function
generateExternalFunction(pathOpenSimModel, outputDir, jointsOrder,...
    coordinatesOrder, input3DBodyForces, input3DBodyMoments,...
    export3DPositions, export3DVelocities, exportGRFs,...
    exportGRMs, exportSeparateGRFs, exportContactPowers, outputFilename, compiler,...
    verbose_mode, verify_ID, secondOrderDerivatives, noDll);
