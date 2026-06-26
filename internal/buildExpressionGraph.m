function [] = buildExpressionGraph(pathRecorderSource, dirRecorderBuild,...
    pathOpenSimAD_install, generator, build_type, verbosityLevel)
% --------------------------------------------------------------------------
% buildExpressionGraph
%   Generates an expression graph and saves it as python code (foo.py)
%
%
% INPUT:
%   - pathRecorderSource -
%   * source code to run AD-recorder (.pp) [char]
%
%   - dirRecorderBuild -
%   * full path to directory where the AD-recorder application should be [char]
%
%   - pathOpenSimAD_install -
%   *
%
%   - generator -
%   * command prompt argument for cmake. [char]
%
%   - buildType -
%   * Build type of the OpenSimAD libraries. [char]
%   'Release','RelWithDebInfo','Debug','MinSizeRel'
%
%   - verbosityLevel -
%   * Controls how much information is printed to the matlab command 
%   window. [int]
%       0: none
%       1: basic
%       2: debug
%
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

workdir = pwd;

%% Prepare information to be passed to cmake

[CPP_DIR, outputFilename,~] = fileparts(pathRecorderSource);


if ispc % Windows

    SDK_DIR = fullfile(pathOpenSimAD_install, 'sdk');
    BIN_DIR = fullfile(pathOpenSimAD_install, 'bin');

    % generator (-G) and platform (-A) arguments for cmake
    if isempty(generator)
        cmake_generator = '-A x64';
    elseif contains(generator, '64')
        cmake_generator = ['-G "', generator, '"'];
    else
        cmake_generator = ['-G "', generator, '" -A x64'];
    end

    % flags to pass to the c++ compiler
    cpp_flags = '/EHsc'; % Exception handling used in opensimAD libraries

    if verbosityLevel < 3
        cpp_flags = [cpp_flags, ' /W0']; % Suppres all compiler warnings.
    end


    pathBuild = fullfile(dirRecorderBuild, outputFilename);
    cd(pathBuild)
    
elseif isunix % linux or macOS

    SDK_DIR = pathOpenSimAD_install;
    BIN_DIR = pathOpenSimAD_install;

    % generator (-G) argument for cmake
    if isempty(generator)
        cmake_generator = '-G "Unix Makefiles"';
    else
        cmake_generator = ['-G "', generator, '"'];
    end

    % flags to pass to the c++ compiler
    cpp_flags = '';
    
end



%% Use cmake to build application that runs AD-recorder for the OpenSim model

cmd_config = ['cmake "' dirRecorderBuild '" ',...
        cmake_generator,...
        ' -DCMAKE_CXX_FLAGS="',cpp_flags, '"',... 
        ' -DTARGET_NAME:STRING="', outputFilename, '"',...
        ' -DSDK_DIR:PATH="' SDK_DIR '" -DCPP_DIR:PATH="' CPP_DIR '"'];

cmd_build = ['cmake --build . --config ', build_type];

if verbosityLevel >= 2
    fprintf("Configuring CMake\n")
    system(cmd_config);
    fprintf("\nBuilding\n")
    system(cmd_build);

else
    [~,~] = system(cmd_config);
    [~,~] = system(cmd_build);

end



%% Run application that runs AD-recorder for the OpenSim model

%
if verbosityLevel >= 2
    fprintf("\nRunning AD-recorder\n")
end

try
    
    if ispc % Windows
        cd(BIN_DIR); % So windows knows where the binaries are
        system(['"' fullfile(pathBuild, build_type, [outputFilename '.exe']) '"']);
        cd(workdir)

    elseif isunix % macOS or Linux
        system(['"' fullfile(pathBuild, outputFilename) '"']);
    end

catch ME
    % clean-up
    cd(workdir)

    % error
    rethrow(ME)
end

delete(lockFile)


end % end of function
