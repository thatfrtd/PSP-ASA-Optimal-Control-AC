function [ResultsObj, results, result_arrays] = ReadControllerTestResults(test_file_name,load_results,result_file_name,result_signals)
arguments
    test_file_name
    load_results
    result_file_name
    result_signals
end
% Assumes the controller type variable is the first one that was overrode by
% the test (won't work if this is not true)

if load_results
    open(result_file_name);
    rsList = sltest.testmanager.getResultSets;
    ResultsObj = rsList(end);
else
    tc = getAllTestCases(sltest.testmanager.TestFile(test_file_name));
    
    % Run the test case and return an object with results data
    ResultsObj = run(tc);
end

% Get the test case result and the Sim Output run dataset
tcr = getTestCaseResults(ResultsObj);
tir = getIterationResults(tcr);

results = [];
result_arrays = {};

variable_names = string({tir(1).IterationSettings.variableParameters.parameterName});

last_controller_type = "";
t_i = 1;
for t = 1:numel(tir)
    controller_type = tir(t).IterationSettings.variableParameters(1).value;
    if controller_type ~= last_controller_type
        t_i = 1;
        
        for s = 1:numel(result_signals)
            result_arrays.(result_signals(s)).(string(controller_type)) = {};
        end

        for v = 2:numel(variable_names)
            results.(string(controller_type)).(variable_names(v)) = [];
        end

        last_controller_type = string(controller_type);
    end
    results.(string(controller_type)).outcome(t_i) = tir(t).Outcome;
    for v = 2:numel(variable_names)
        results.(string(controller_type)).(variable_names(v))(t_i) = tir(t).IterationSettings.variableParameters(v).value;
    end
    stop_time = getOutputRuns(tir(t)).StopTime;
    
    if results.(string(controller_type)).outcome(t_i) == sltest.testmanager.TestResultOutcomes.Disabled || results.(string(controller_type)).outcome(t_i) == sltest.testmanager.TestResultOutcomes.Failed
        stop_time = nan;
    end

    results.(string(controller_type)).stop_time(t_i) = stop_time;

    % Resample so can just have array instead of cells? More convenient...
    for s = 1:numel(result_signals)
        result_array = getSignalsByName(getOutputRuns(tir(t)),result_signals(s));
        result_arrays.(result_signals(s)).(string(controller_type))(t_i) = {[result_array(1).Values.Time, result_array(1).Values.Data]};
    end

    t_i = t_i + 1;
end
end