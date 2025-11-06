function [] = notcmake(pathOutputFile, pathBuild, outputFilename, sdk_dir, verbose_mode)

sourceFile = [pathOutputFile,'.cpp'];
outFile = fullfile(pathBuild,[outputFilename,'.exe']);

[root_dir,~,~] = fileparts(sdk_dir);
bin_dir = fullfile(root_dir,'bin');

includeDirs = {fullfile(sdk_dir,'include'),...
    fullfile(sdk_dir,'Simbody/include')};

compileDefinitions = {'SimTK_REAL_IS_ADOUBLE'};

linkLibraries = {fullfile(sdk_dir,'lib/osimCommon_recorder.lib'),...
    fullfile(sdk_dir,'lib/osimSimulation_recorder.lib'),...
    fullfile(sdk_dir,'Simbody/lib/SimTKcommon_recorder.lib'),...
    fullfile(sdk_dir,'Simbody/lib/SimTKmath_recorder.lib'),...
    fullfile(sdk_dir,'Simbody/lib/SimTKsimbody_recorder.lib')};

% linkLibraries = {'osimCommon_recorder.lib',...
%     'osimSimulation_recorder.lib',...
%     'SimTKcommon_recorder.lib',...
%     'SimTKmath_recorder.lib',...
%     'SimTKsimbody_recorder.lib'};
% 
% libDirs = {fullfile(sdk_dir,'lib'), fullfile(sdk_dir,'Simbody/lib'),...
%     bin_dir};

%% GNU compiler
% compiler_str = 'g++ -std=c++11';
% 
% for i=1:length(compileDefinitions)
%     compiler_str = [compiler_str, ' -D', char(compileDefinitions{i})];
% end
% 
% compiler_str = [compiler_str, ' -o "', replace(char(outFile),'\','/'), '"'];
% 
% compiler_str = [compiler_str, ' "', replace(char(sourceFile),'\','/'), '"'];
% 
% for i=1:length(linkLibraries)
%     compiler_str = [compiler_str, ' "', replace(char(linkLibraries{i}),'\','/'), '"'];
% end
% 
% 
% for i=1:length(includeDirs)
%     compiler_str = [compiler_str, ' -I "', replace(char(includeDirs{i}),'\','/'), '"'];
% end
% 
% for i=1:length(libDirs)
%     compiler_str = [compiler_str, ' -L "', replace(char(libDirs{i}),'\','/'), '"'];
% end

%% visual studio compiler
compiler_str = 'cl /std:c++14 /W0 /EHsc';
compiler_str = [compiler_str, ' /Fo:"', replace(char(pathBuild),'\','/'), '/"'];

% compiler_str = 'icx /std:c++14 /W0 /EHsc';

for i=1:length(compileDefinitions)
    compiler_str = [compiler_str, ' /D', char(compileDefinitions{i})];
end

compiler_str = [compiler_str, ' /Fe:"', replace(char(outFile),'\','/'), '"'];

for i=1:length(linkLibraries)
    compiler_str = [compiler_str, ' "', replace(char(linkLibraries{i}),'\','/'), '"'];
end

for i=1:length(includeDirs)
    compiler_str = [compiler_str, ' /I "', replace(char(includeDirs{i}),'\','/'), '"'];
end

compiler_str = [compiler_str, ' "', replace(char(sourceFile),'\','/'), '"'];

%%
if verbose_mode
    system(compiler_str);
else
    [~,~] = system(compiler_str);
end

end
