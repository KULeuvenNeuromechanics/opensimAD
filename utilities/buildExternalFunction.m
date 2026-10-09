function [] = buildExternalFunction(fooPath, outputFilename, outputDir, compiler, verbose_mode, buildRoot)
% --------------------------------------------------------------------------
% buildExternalFunction
%   Compiles the expression graph (foo_jac.c) into an external function (.dll)
%
%
% INPUT:
%   - fooPath -
%   * path to foo.py and foo_jac.c [char]
%
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

%% Build in a separate short workspace, also when called directly.
% build directory as optional input argument
if nargin < 6
    buildRoot = [];
end
% path information
oldDir = pwd;
restoreDir = onCleanup(@() cd(oldDir));
[pathUtilities,~,~] = fileparts(mfilename('fullpath'));
[pathMain,~,~] = fileparts(pathUtilities);
pathBuild = createBuildWorkspace(buildRoot, 'buildExternalFunction');
pathInstall = fullfile(pathBuild, 'install');
% copy files to build location
copyfile(fullfile(pathMain, 'buildExternalFunction', 'CMakeLists.txt'), pathBuild);
copyfile(fullfile(fooPath, 'foo_jac.c'), pathBuild);
cd(pathBuild);
% Keep the public DLL name in the import library, but use a short logical
% target name for Visual Studio intermediate files.
command = ['cmake . -G "' compiler '" -DOUTPUT_NAME:STRING="' ...
    outputFilename '" -DINSTALL_DIR:PATH="' pathInstall '"'];
runBuildCommand(command, 'External function configuration', verbose_mode);
runBuildCommand('cmake --build . --config RelWithDebInfo --target install', ...
    'External function compilation and installation', verbose_mode);
pathDll = fullfile(pathInstall, 'bin', [outputFilename '.dll']);
pathLib = fullfile(pathInstall, 'lib', [outputFilename '.lib']);
% error messages when build files were not generated
requireBuildFile(pathDll, 'External function compilation');
requireBuildFile(pathLib, 'External function compilation');
% copy files to desired directory
cd(oldDir);
copyfile(pathDll, outputDir);
copyfile(pathLib, outputDir);
% clean up build dir (only after success)
cleanupBuildWorkspace(pathBuild, oldDir);
end
