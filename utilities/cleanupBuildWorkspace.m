function cleanupBuildWorkspace(pathBuild, originalDir)
% Leave the build directory before removing it after a successful build.
cd(originalDir);
if isfolder(pathBuild)
    rmdir(pathBuild, 's');
end
end
