function [] = generateExternalFunction(pathOpenSimModel, outputDir,...
    jointsOrder, coordinatesOrder, input3DBodyForces, input3DBodyMoments,...
    export3DPositions, export3DVelocities,...
    exportGRFs, exportGRMs, exportSeparateGRFs, exportContactPowers,...
    outputFilename, compiler, verboseMode, verify_ID,...
    secondOrderDerivatives, noDll)
% --------------------------------------------------------------------------
% generateExternalFunction
%   This function uses OpenSimAD to generate a CasADi external function. 
%   Given an OpenSim model provided as an .osim file, this script generates
%   a C++ file with a function F building the musculoskeletal model 
%   programmatically and running inverse dynamics. The C++ file is then 
%   compiled as an .exe, which when run generates the expression graph 
%   underlying F. From this expression graph, CasADi can generate C code 
%   containing the function F and its Jacobian in a format understandable 
%   by CasADi. This code is finally compiled as a .dll that can be imported 
%   when formulating trajectory optimization problems with CasADi.
%   The expression graph is also serialised and saved to a file (.casadi),
%   which can be loaded as a CasADi Function in MATLAB/python directly. 
%   The serialised file is not compatible with older versions of CasADi,
%   while the external function is.
%
%   The function F takes as:
%       - INPUTS: 
%           - joint positions and velocities (intertwined)
%           - joint accelerations
%           - (optional) forces acting on bodies (OpenSim StationForce)
%           - (optional) torques acting on bodies (OpenSim BodyTorque)
%       - OUTPUTS:
%           - joint torques (inverse dynamics)
%           - (optional) position of a point (OpenSim pointkinematics)
%           - (optional) velocity of a point (OpenSim pointkinematics)
%           - (optional) total ground reaction forces of left and right side.
%           Contact elements are identified as left or right based on their 
%           name having a prefix (r_, R_, l_, L_) or suffix (_r, _R, _l, _L).
%           - (optional) ground reaction moments of left and right side
%           - (optional) ground reaction forces of each contact element
%           - (optional) power due to deformation of each contact element.
%           This includes only the power of the force normal to the ground 
%           plane (visco-elastic), not of the in-plane components (friction).
%           
%   The optional inputs and outputs of F are configured via the arguments
%   below:
%
%
% INPUT:
%   - pathOpenSimModel -
%   * full path to OpenSim model file (.osim) [char]
%
%   - outputDir -
%   * full path to directory where the generated file should be saved [char]
%
%   - outputFilename -
%   * name of the generated file [char]
%
%   - jointsOrder -
%   * names of joints in order they should appear in the external function
%   input/output. Pass empty to use order they are in the model file. 
%   [cell array of char]
%
%   - coordinatesOrder -
%   * names of coordinate in order they should appear in the external 
%   function input/output. Order should be consistent with jointsOrder.
%   Pass empty to use order they are in the model file. [cell array of char]
%
%   - input3DBodyForces -
%   * define inputs to add forces acting on bodies. Forces are expressed as 
%   [x, y, z] components in given reference frame. [array of structs] 
%   Example input:
%     input3DBodyForces(1).body = 'torso';
%     input3DBodyForces(1).point_in_body = [-0.1, 0.3, 0];
%     input3DBodyForces(1).name = 'back_push';
%     input3DBodyForces(1).reference_frame = 'ground';
% 
%   - input3DBodyMoments -
%   * define inputs to add moments acting on bodies. Moments are expressed as 
%   [x, y, z] components in given reference frame. [array of structs] 
%   Example input:
%     input3DBodyMoments(1).body = 'tibia_l';
%     input3DBodyMoments(1).name = 'exo_shank_l';
%     input3DBodyMoments(1).reference_frame = 'tibia_l';
%     input3DBodyMoments(2).body = 'calcn_l';
%     input3DBodyMoments(2).name = 'exo_foot_l';
%     input3DBodyMoments(2).reference_frame = 'tibia_l';
%
%   - export3DPositions -
%   * points of which the position in ground frame should be exported. 
%   [array of structs] Example input:
%       export3DPositions(1).body = 'tibia_l';
%       export3DPositions(1).point_in_body = [0, -0.012, 0];
%       export3DPositions(1).name = 'left_shin';
%       export3DPositions(2).body = 'tibia_r';
%       export3DPositions(2).point_in_body = [0, -0.012, 0];
%       export3DPositions(2).name = 'right_shin';
%
%   - export3DVelocities -
%   * points of which the velocity in ground frame should be exported. 
%   [array of structs] Example input:
%       export3DVelocities(1).body = 'tibia_l';
%       export3DVelocities(1).point_in_body = [0, -0.012, 0];
%       export3DVelocities(1).name = 'left_shin';
%
%   - exportGRFs -
%   * export total ground reaction force of left and right side. [bool]
%
%   - exportGRMs -
%   * export total ground reaction moment of left and right side. [bool]
%
%   - exportSeparateGRFs -
%   * export ground reaction force of each contact element. [bool]
%
%   - exportContactPowers -
%   * export deformation power of each contact element. [bool]
%
%   - compiler -
%   * command prompt argument for the compiler. [char]
%   Example inputs:
%       Visual studio 2015: 'Visual Studio 14 2015 Win64'
%       Visual studio 2017: 'Visual Studio 15 2017 Win64'
%       Visual studio 2017: 'Visual Studio 16 2019'
%       Visual studio 2017: 'Visual Studio 17 2022'
%
%   - verboseMode -
%   * outputs from windows command prompt are printed to matlab command 
%   window if true. [bool]
%
%   - verify_ID -
%   * the generated function is verified versus the inverse dynamics tool
%   in OpenSim if true. [bool]
%
%   - secondOrderDerivatives -
%   * The generated library always contains the expression graphs to evaluate
%   the Jacobian of the external function. If this input is true, expression
%   graphs for evaluating second derivative information are also added. Do 
%   note that this greatly increases the compiling time, especially for models
%   with many degrees of freedom. [bool]
%
%   - noDll -
%   * if true, no .dll (or .lib) files are generated. [bool]
%
% OUTPUT:
%   This function does not return outputs, but generates files. Assuming 
%   outputFilename = 'filename', the following files are saved in the folder 
%   given by outputDir. 
%   - filename.dll -
%   * file containing the CasADi external function. To get the function in
%   matlab, use: F = external('F','filename.dll')
%   This function takes a column vector as input, and returns a column 
%   vector as output. For more info on external functions, 
%   see https://web.casadi.org/docs/#using-the-generated-code
%
%   - filename.cpp -
%   * source code for the .dll, you do not need this.
%
%   - filename.lib -
%   * if you want to compile code that calls filename.dll, you need this.
%
%   - filename_IO.mat -
%   * contains a struct (IO) where the fieldnames denote an output of the
%   generated function, and the numeric values are the corresponding
%   indices of the output vector of the generated function.
%   The input indices can be contructed as:
%       positions: IO.coordi.(name)*2-1
%       velocities: IO.coordi.(name)*2
%       acceleration: IO.coordi.(name) * IO.nCoordinates*2
%
%   - filename.casadi -
%   * serialised CasADi Function. Can be loaded into MATLAB via 
%       F = Function.load('filename.casadi');
%   Note that not every version of CasADi can load this Function. 
%   v3.6.3 is confirmed to work, v3.5.5 is confirmed to not work
% 
%
% Note: 
%   This code ignores the contribution of the patella to the inverse
%   dynamics. Assuming the patella bodies are named 'patella_l' and
%   'patella_r', the joint names include 'patel', and the coordinate names
%   are 'knee_angle_l_beta' and 'knee_angle_r_beta'.
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

%% Write the cpp file.
writeCppFile(pathOpenSimModel, outputDir, outputFilename,...
    jointsOrder, coordinatesOrder, input3DBodyForces, input3DBodyMoments,...
    export3DPositions, export3DVelocities,...
    exportGRFs, exportGRMs, exportSeparateGRFs, exportContactPowers);

%% Build expression graph (foo.py)
[fooPath] = buildExpressionGraph(outputFilename, outputDir, compiler, verboseMode);

%% Generate code with expression graph and derivative information (foo_jac.c)
load(fullfile(outputDir, [outputFilename, '_IO.mat']),'IO');
generateF(IO.input.nInputs, fooPath, secondOrderDerivatives);

%% Copy serialised casadi Function
if exist(fullfile(fooPath,'F_foo.casadi'),'file')
    copyfile(fullfile(fooPath,'F_foo.casadi'), fullfile(outputDir,[outputFilename,'.casadi']));
end

%% Build external Function (.dll file).
if ~noDll
    buildExternalFunction(fooPath, outputFilename, outputDir, compiler, verboseMode);
end

%% Verification
% Run ID with the .osim file and verify that we can get the same torques as
% with the external function.
if verify_ID
    VerifyInverseDynamics(pathOpenSimModel, outputDir, outputFilename, verboseMode);
end

%% Clean-up
removeAllTempFiles(outputFilename);

end
