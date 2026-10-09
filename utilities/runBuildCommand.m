function runBuildCommand(command, stage, verboseMode)
% Keep diagnostics even when routine command output is suppressed.
[status, output] = system(command);
if verboseMode
    fprintf('%s', output);
end
if status ~= 0
    error('opensimAD:CommandFailed', ...
        '%s failed (exit code %d).\nCommand: %s\n%s', ...
        stage, status, command, output);
end
end
