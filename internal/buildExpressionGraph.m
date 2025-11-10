function [] = buildExpressionGraph(pathOutputFile, pathRecorderStream,...
    pathBuildExpressionGraph, pathOpenSimAD_install, generator, verbose_mode)
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
%   - generator -
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
workdir = pwd;


[CPP_DIR, outputFilename,~] = fileparts(pathOutputFile);

pathBuild = fullfile(pathBuildExpressionGraph, outputFilename);

if ispc

    SDK_DIR = fullfile(pathOpenSimAD_install, 'sdk');
    BIN_DIR = fullfile(pathOpenSimAD_install, 'bin');

    if isempty(generator)
        cmake_generator = '-A x64';
    else
        cmake_generator = ['-G "',generator, '"'];
    end

    cmd1 = ['cmake "' pathBuildExpressionGraph '" ', cmake_generator,...
        ' -DTARGET_NAME:STRING="', outputFilename, '"',...
        ' -DCMAKE_CXX_FLAGS="/W0 /EHsc"',...
        ' -DSDK_DIR:PATH="' SDK_DIR '" -DCPP_DIR:PATH="' CPP_DIR '"'];
    cmd2 = 'cmake --build . --config Release';

elseif ismac

elseif isunix

    SDK_DIR = pathOpenSimAD_install;
    BIN_DIR = pathBuild;

    cmd1 = ['cmake "' pathBuildExpressionGraph '"',...
        ' -DTARGET_NAME:STRING="', outputFilename '"' ...
        ' -DSDK_DIR:PATH="' SDK_DIR '" -DCPP_DIR:PATH="' CPP_DIR '"'
        ' -DCMAKE_BUILD_TYPE=Release'];
    cmd2 = 'make';
    

end


%% use cmake to compile .cpp to .exe
cd(pathBuild);
if verbose_mode
    system(cmd1);
else
    [~,~] = system(cmd1);
end

if verbose_mode
    system(cmd2);
else
    [~,~] = system(cmd2);
end

%% run .exe to generate foo.py
% Recorder does not work when running multiple opensimAD instances in 
% parallel. To prevent this, we use a file (lockFile.txt) to indicate when 
% recorder is busy.
lockFile = fullfile(BIN_DIR,'lockFile.txt');
isLocked = isfile(lockFile);
t0 = tic;

while isLocked
    isLocked = isfile(lockFile);
    pause(10)

    if toc(t0) > 300
        error(['OpenSimAD timed out. Another instance of OpenSimAD took too ',...
            'long, or failed to delete its lockFile when done.'])
    end
end

fid = fopen(lockFile,'w');
fprintf(fid, ['Recorder is running for ' outputFilename '.']);
fprintf(fid, 'This file will be deleted after Recorder finished.');
fprintf(fid, ['Start: ' datestr(datetime,0)]);
fclose(fid);

try
    cd(BIN_DIR);
    if ispc
        path_EXE = fullfile(pathBuild, 'Release', [outputFilename '.exe']);
    elseif isunix
        path_EXE = fullfile(pathBuild, outputFilename);
    end

    system(['"' path_EXE '"']);

catch ME
    % clean-up
    delete(lockFile)
    cd(workdir)

    % error
    rethrow(ME)
end

delete(lockFile)
cd(workdir)

end
