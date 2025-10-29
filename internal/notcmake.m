function [] = notcmake(pathOutputFile, pathBuild, outputFilename, sdk_dir)

sourceFile = [pathOutputFile,'.cpp'];
outFile = fullfile(pathBuild,[outputFilename,'.exe']);

includeDirs = {fullfile(sdk_dir,'include'),...
    fullfile(sdk_dir,'Simbody/include')};

compileDefinitions = {'SimTK_REAL_IS_ADOUBLE'};

linkLibraries = {fullfile(sdk_dir,'lib/osimCommon_recorder.lib'),...
    fullfile(sdk_dir,'lib/osimSimulation_recorder.lib'),...
    fullfile(sdk_dir,'Simbody/lib/SimTKcommon_recorder.lib'),...
    fullfile(sdk_dir,'Simbody/lib/SimTKmath_recorder.lib'),...
    fullfile(sdk_dir,'Simbody/lib/SimTKsimbody_recorder.lib')};


compiler_str = 'icpx';

for i=1:length(compileDefinitions)
    compiler_str = [compiler_str, ' /D', char(compileDefinitions{i})];
end

compiler_str = [compiler_str, ' /Fe"', replace(char(outFile),'\','/'), '"'];

compiler_str = [compiler_str, ' "', replace(char(sourceFile),'\','/'), '"'];

for i=1:length(linkLibraries)
    compiler_str = [compiler_str, ' "', replace(char(linkLibraries{i}),'\','/'), '"'];
end


for i=1:length(includeDirs)
    compiler_str = [compiler_str, ' /I"', replace(char(includeDirs{i}),'\','/'), '"'];
end



system(compiler_str);

end
