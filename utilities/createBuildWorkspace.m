function pathBuild = createBuildWorkspace(root, stage)
% One short, unique directory per conversion, independent of the model name.
if isempty(root)
    pathMain = fileparts(fileparts(mfilename('fullpath')));
    root = fullfile(pathMain, stage);
end
if ~isfolder(root)
    error('opensimAD:BuildRoot', 'Build root does not exist: %s', root);
end
oldDir = pwd;
restoreDir = onCleanup(@() cd(oldDir));
cd(root);
root = pwd; % Resolve relative roots before other stages change directory.
pathBuild = tempname(root);
% Reserve space for Visual Studio target, configuration and tracking paths.
if ispc && length(pathBuild) + 120 >= 260
    error('opensimAD:BuildRootTooLong', ...
        'Build root is too long: %s. Pass a shorter existing directory as the optional buildRoot argument.', root);
end
mkdir(pathBuild);
fprintf('Build directory (retained on failure): %s\n', pathBuild);
end
