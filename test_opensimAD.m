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
opts.verbosityLevel = 2;
% opts.generator = 'Visual Studio 17 2022';
% opts.generator = 'Visual Studio 15 2017 Win64';
% opts.buildType = 'RelWithDebInfo';

opts.pathOpenSimAD_install = 'C:\GBW_MyPrograms\OpenSimAD-lib\opensim-ad-core-install';

generateADFunction(pathOpenSimModel, outputDir, outputFilename, opts);




