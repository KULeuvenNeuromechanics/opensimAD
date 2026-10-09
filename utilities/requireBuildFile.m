function requireBuildFile(path, stage)
if ~isfile(path)
    error('opensimAD:MissingArtifact', '%s did not produce the expected file: %s', stage, path);
end
end
