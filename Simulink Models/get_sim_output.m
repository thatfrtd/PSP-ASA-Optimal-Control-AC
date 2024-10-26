function [outputs] = get_sim_output(simOut, options)
%GET_SIM_OUTPUT Extract outputs from a Simulink output object
%   Convenience function for getting a struct of simulation outputs from a
%   Simulink SimulationOutput object. If no outputs are given for
%   requested_outputs, it returns all of the output variables.
arguments
    simOut 
    options.requested_outputs = []
end

% Extract Outputs
if isempty(options.requested_outputs)
    options.requested_outputs = string(getElementNames(simOut.yout));
end

outputs = struct();

% Extract simulation outputs
outputs.time = simOut.tout;
for o = 1:length(options.requested_outputs)
    name = options.requested_outputs(o);
    
    outputs.(name) = squeeze(simOut.yout.get(name).Values.Data);
end
end

