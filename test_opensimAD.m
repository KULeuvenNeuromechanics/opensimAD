clear
clc


% Get current directory
pathMain = pwd;

% Provide path to OpenSim model.
pathOpenSimModel = fullfile(pathMain, 'examples', 'double_pendulum.osim');

% Path to folder for outputs
outputDir = fullfile(pathMain, 'examples');

% Output file name
outputFilename = 'F_2pend';

% Compiler
% opts.generator = 'Visual Studio 17 2022';
opts.pathOpenSimAD_install = 'C:\GBW_MyPrograms\opensimAD-core\core-install-v3';

generateADFunction(pathOpenSimModel, outputDir, outputFilename, opts);




