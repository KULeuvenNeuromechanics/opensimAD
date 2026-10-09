function [pathFoo] = buildExpressionGraph(outputFilename,...
    outputDir, compiler, verbose_mode, buildRoot)
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
%   * caller owns this temporary folder and must remove it after use
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
if nargin < 5, buildRoot = []; end
oldDir = pwd;
restoreDir = onCleanup(@() cd(oldDir));
[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathUtilities);
pathBuildExpressionGraph = fullfile(pathMain, 'buildExpressionGraph');
pathBuild = createBuildWorkspace(buildRoot, 'buildExpressionGraph');
pathFoo = pathBuild;
try
    copyfile(fullfile(outputDir, [outputFilename '.cpp']), fullfile(pathBuild, 'graph.cpp'));
    copyfile(fullfile(pathBuildExpressionGraph, 'CMakeLists.txt'), pathBuild);
    SDK_DIR = fullfile(pathMain, 'OpenSimAD-install', 'sdk');
    BIN_DIR = fullfile(pathMain, 'OpenSimAD-install', 'bin');
    cd(pathBuild);
    command = ['cmake . -G "' compiler '" -DSDK_DIR:PATH="' SDK_DIR '"'];
    runBuildCommand(command, 'Expression graph configuration', verbose_mode);
    runBuildCommand('cmake --build . --config RelWithDebInfo', ...
        'Expression graph compilation', verbose_mode);
    pathEXE = fullfile(pathBuild, 'RelWithDebInfo', 'osimad_graph.exe');
    requireBuildFile(pathEXE, 'Expression graph compilation');

    % The recorder writes foo.py in the SDK bin directory. Preserve the
    % existing serialization; only release the lock acquired by this call.
    lockFile = fullfile(BIN_DIR, 'lockFile.txt');
    pathBinFoo = fullfile(BIN_DIR, 'foo.py');
    t0 = tic;
    while isfile(lockFile)
        if toc(t0) > 300
            error('opensimAD:LockTimeout', ...
                ['opensimAD timed out. Another instance of opensimAD took too ', ...
                'long to generate foo.py, or failed to delete its lockFile when done.\n', ...
                'Lock file: %s'], lockFile);
        end
        pause(1);
    end
    fid = fopen(lockFile, 'w');
    if fid < 0
        error('opensimAD:LockFile', 'Cannot create lock file: %s', lockFile);
    end
    fprintf(fid, 'Generating foo.py for %s.', outputFilename);
    fclose(fid);
    releaseLock = onCleanup(@() cleanupRecorder(pathBinFoo, lockFile));
    if isfile(pathBinFoo)
        delete(pathBinFoo); % Do not mistake a stale graph for new output.
    end
    cd(BIN_DIR);
    runBuildCommand(['"' pathEXE '"'], 'Expression graph execution', verbose_mode);
    requireBuildFile(pathBinFoo, 'Expression graph execution');
    copyfile(pathBinFoo, pathFoo);
    clear releaseLock;
    cd(oldDir);
catch ME
    cd(oldDir);
    rethrow(ME);
end
end

function cleanupRecorder(pathFoo, lockFile)
if isfile(pathFoo)
    delete(pathFoo);
end
if isfile(lockFile)
    delete(lockFile);
end
end
