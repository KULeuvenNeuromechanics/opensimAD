function [] = VerifyInverseDynamics(pathOpenSimModel, outputDir, outputFilename, verbosityLevel)
% --------------------------------------------------------------------------
% VerifyInverseDynamics
%   Compare the inverse dynamics outputs of the external function versus
%   OpenSim ID Tool.
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
%   - verbosityLevel -
%   * Controls how much information is printed to the matlab command 
%   window. [int]
%       0: none
%       1: basic
%       2: debug
%
%
% OUTPUT:
%   - (This function does not return output arguments) -
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


import org.opensim.modeling.*;
import casadi.*

[pathInternal,~,~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathInternal);
pathID = fullfile(pathMain, 'intermediateFiles', 'InverseDynamics');

[~,osimFileName,~] = fileparts(pathOpenSimModel);

% load input/output indices information
load(fullfile(outputDir, [outputFilename, '_IO.mat']),'IO');

coordinatesOrder = fieldnames(IO.coordi);
all_coordi = IO.coordi;
joint_isTra = IO.jointi.translations;
nCoordinates = length(coordinatesOrder);

% create state values
vec1 = zeros(IO.input.nInputs, 1);
vec1(1:2:2*nCoordinates) = 0.05;
if isfield(IO.input.Qs, 'pelvis_ty')
    vec1(IO.input.Qs.pelvis_ty) = -0.05;
end

%% Run ID with the .osim file 

% Generate .mot file with same position inputs
mot_file = ['Verify_', outputFilename, '.mot'];
path_mot = fullfile(pathID, outputFilename, mot_file);

if ~isfolder(fullfile(pathID, outputFilename))
    mkdir(fullfile(pathID, outputFilename))
end

if ~exist(path_mot, 'file')
    labels = [{'time'}, coordinatesOrder'];
    vec4 = vec1(1:2:2*nCoordinates);
    data_coords = repmat(vec4, 1, 10);
    data_time = zeros(1, 10);
    data_time(1, :) = 0.01:0.01:0.1;
    data = [data_time', data_coords'];

    q.labels = labels;
    q.data = data;
    q.inDeg = 'no';
    write_motionFile_v40(q, path_mot)
end

% ID tool setup
pathGenericIDSetupFile = fullfile(pathID, 'SetupID.xml');
idTool = InverseDynamicsTool(pathGenericIDSetupFile);
idTool.setName('ID_withOsimAndIDTool');
idTool.setModelFileName(pathOpenSimModel);
idTool.setResultsDir(fullfile(pathID, outputFilename));
idTool.setCoordinatesFileName(path_mot);
idTool.setOutputGenForceFileName('ID_withOsimAndIDTool.sto');
pathSetupID = fullfile(pathID, outputFilename, 'SetupID.xml');
idTool.print(pathSetupID);

[~,~] = system(['opensim-cmd run-tool "', pathSetupID, '"']);


% Extract torques from .osim + ID tool.
data = importdata(fullfile(pathID, outputFilename, 'ID_withOsimAndIDTool.sto'));

ID_osim = zeros(nCoordinates, 1);
for count = 1:numel(coordinatesOrder)
    coordinateOrder = coordinatesOrder{count};
    if any(all_coordi.(coordinateOrder) == joint_isTra)
        suffix_header = '_force';
    else
        suffix_header = '_moment';
    end
    ID_osim(count) = data.data(1,strcmp(data.colheaders,[coordinateOrder,suffix_header]));

end



%% Compare torques from external function.
funPath = replace(fullfile(outputDir, [outputFilename, '.casadi']),'\','/');
if isfile(funPath)

    F = casadi.Function.load(funPath);

    ID_F = full(F(vec1));
    ID_F = ID_F(1:nCoordinates);
    
    % Assert we get the same torques.
    test_diff = max(abs(ID_osim - ID_F)) < 1e-6;
    if test_diff
        if verbosityLevel >= 1
            disp(['Inverse dynamics from "', outputFilename,...
                '.casadi" matches IDTool for "', osimFileName '.osim".'])
        end
    else
        warning(['Inverse dynamics from "', outputFilename,...
            '.casadi" does not match IDTool for "', osimFileName '.osim".']);
    end

end


end