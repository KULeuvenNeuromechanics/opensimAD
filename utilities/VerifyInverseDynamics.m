function [] = VerifyInverseDynamics(pathOpenSimModel, outputDir, outputFilename, verbose_mode)
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
%   - verbose_mode -
%   * outputs from windows command prompt are printed to matlab command 
%   window if true. [bool]
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

[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathUtilities);
pathID = fullfile(pathMain, 'InverseDynamics');

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
model = Model(pathOpenSimModel);
model.initSystem();
coordinateSet = model.getCoordinateSet();

% Generate .mot file with same position inputs
mot_file = ['Verify_', outputFilename, '.mot'];
path_mot = fullfile(pathID, mot_file);

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

pathGenericIDSetupFile = fullfile(pathID, 'SetupID.xml');
idTool = InverseDynamicsTool(pathGenericIDSetupFile);
idTool.setName('ID_withOsimAndIDTool');
idTool.setModelFileName(pathOpenSimModel);
idTool.setResultsDir(outputDir);
idTool.setCoordinatesFileName(path_mot);
idTool.setOutputGenForceFileName('ID_withOsimAndIDTool.sto');
pathSetupID = fullfile(outputDir, 'SetupID.xml');
idTool.print(pathSetupID);

command = ['opensim-cmd', ' run-tool ', '"' pathSetupID '"'];
if verbose_mode
    system(command);
else
    [~,~] = system(command);
end

% Extract torques from .osim + ID tool.
data = importdata(fullfile(outputDir, 'ID_withOsimAndIDTool.sto'));

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
dllPath = replace(fullfile(outputDir, [outputFilename, '.dll']),'\','/');
if isfile(dllPath)
    F = external('F', dllPath);
    ID_F = full(F(vec1));
    ID_F = ID_F(1:nCoordinates);
    
    % Assert we get the same torques.
    test_diff = max(abs(ID_osim - ID_F)) < 1e-6;
    if test_diff
        disp(['Inverse dynamics from "', outputFilename,...
            '.dll" matches IDTool for "', osimFileName, '.osim".'])
    else
        warning(['Inverse dynamics from "', outputFilename,...
            '.dll" does not match IDTool for "', osimFileName, '.osim".']);
    end
 
end

%% Compare torques from external function.
funPath = replace(fullfile(outputDir, [outputFilename, '.casadi']),'\','/');
if isfile(funPath)
    try
        cd(outputDir);
        F = casadi.Function.load(funPath);
    catch cas_e
        warning(['Tried to load "',funPath,...
            '" with CasADi libraries located in "',GlobalOptions.getCasadiPath(),'".']);
        rethrow(cas_e)
    end
    ID_F = full(F(vec1));
    ID_F = ID_F(1:nCoordinates);
    
    % Assert we get the same torques.
    test_diff = max(abs(ID_osim - ID_F)) < 1e-6;
    if test_diff
        disp(['Inverse dynamics from "', outputFilename,...
            '.casadi" matches IDTool for "', osimFileName '.osim".'])
        
    else
        warning(['Inverse dynamics from "', outputFilename,...
            '.casadi" does not match IDTool for "', osimFileName '.osim".']);
    end

end

%% Clean-up
delete(fullfile(outputDir, 'ID_withOsimAndIDTool.sto'));
delete(path_mot);
delete(pathSetupID);

end