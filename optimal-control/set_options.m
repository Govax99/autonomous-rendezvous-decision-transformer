function defaultOptionsValues = set_options(optionsNames,optionsValues)

supportedOptions = ["-ntot", "-nsave", "-printlevel", "-savedir", "-rngseed"];
defaultOptionsValues = {10000, 500, 0, "results", 42}; % because matlab does not have dictionaries
if (length(optionsNames) ~= length(optionsValues))
    error("Different number of option names and values.")
end

for i = 1:length(optionsNames)
    if ~any(supportedOptions == optionsNames(i))
       error("Unknown input option")
    end
end

% 2) default simulation settings or inputted
for i = 1:length(supportedOptions)
    ind = find(supportedOptions(i) == optionsNames, 1);
    if ~isempty(ind)
        defaultOptionsValues(i) = optionsValues(ind);
    end
end
end

