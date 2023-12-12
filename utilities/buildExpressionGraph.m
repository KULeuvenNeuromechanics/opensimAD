function [pathFoo] = buildExpressionGraph(outputFilename,...
    outputDir, compiler, verbose_mode)
% --------------------------------------------------------------------------
% buildExpressionGraph
%   Generates an expression graph and saves it as python code (foo.py)
%
%
% INPUT:
%   - outputFilename -
%   * name of the generated file [char]
%
%   - outputDir -
%   * full path to directory where the generated file should be saved [char]
%
%   - compiler -
%   * command prompt argument for the compiler. [char]
%   Example inputs:
%       Visual studio 2015: 'Visual Studio 14 2015 Win64'
%       Visual studio 2017: 'Visual Studio 15 2017 Win64'
%       Visual studio 2017: 'Visual Studio 16 2019'
%       Visual studio 2017: 'Visual Studio 17 2022'
%
%   - verbose_mode -
%   * outputs from windows command prompt are printed to matlab command 
%   window if true. [bool]
%
%
% OUTPUT:
%   - pathFoo -
%   * path to the folder where foo.py is
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

%% set paths
[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathUtilities);
pathBuildExpressionGraph = fullfile(pathMain, 'buildExpressionGraph');
pathBuild = fullfile(pathMain, 'buildExpressionGraph', outputFilename);
mkdir(pathBuild);

OpenSimAD_DIR = fullfile(pathMain, 'OpenSimAD-install');
SDK_DIR = fullfile(OpenSimAD_DIR, 'sdk');
BIN_DIR = fullfile(OpenSimAD_DIR, 'bin');

path_bin_foo = fullfile(BIN_DIR, 'foo.py');
pathFoo = pathBuild;

%% use cmake to compile .cpp to .exe
cd(pathBuild);
cmd1 = ['cmake "', pathBuildExpressionGraph, '" -G "', compiler, '" -DTARGET_NAME:STRING="',...
    outputFilename, '" -DSDK_DIR:PATH="', SDK_DIR, '" -DCPP_DIR:PATH="', outputDir, '"'];
if verbose_mode
    system(cmd1);
else
    [~,~] = system(cmd1);
end
cmd2 = 'cmake --build . --config RelWithDebInfo';
if verbose_mode
    system(cmd2);
else
    [~,~] = system(cmd2);
end

%% run .exe to generate foo.py
% Since it is hardcoded that the generated file is
% ./opensimAD-install/bin/foo.py, this can cause problems when running
% multiple opensimAD instances in parallel. To prevent this, we use a file
% (lockFile.txt) to indicate when opensimAD is generating foo.py.
lockFile = fullfile(BIN_DIR,'lockFile.txt');
isLocked = isfile(lockFile);
t0 = tic;

while isLocked
    isLocked = isfile(lockFile);
    pause(10)

    if toc(t0) > 300
        error(['opensimAD timed out. Another instance of opensimAD took too ',...
            'long to generate foo.py, or failed to delete its lockFile when done.'])
    end
end

fid = fopen(lockFile,'w');
fprintf(fid, ['Generating foo.py for ' outputFilename '.']);
fprintf(fid, 'This file will be deleted after foo.py is generated and copied to its target folder.');
fprintf(fid, ['Start: ' datestr(datetime,0)]);
fclose(fid);

try
    cd(BIN_DIR);
    path_EXE = fullfile(pathBuild, 'RelWithDebInfo', [outputFilename '.exe']);
    system(['"' path_EXE '"']);
    
    copyfile(path_bin_foo, pathFoo);
    delete(path_bin_foo);

catch ME
    % clean-up
    if isfile(path_bin_foo)
        delete(path_bin_foo)
    end
    delete(lockFile)

    % error
    rethrow(ME)
end

delete(lockFile)

end