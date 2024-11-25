%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PSP ASA
% MPC Delay Test Plot Script
% Author: Travis Hastreiter
% Created On: 24 November, 2024
% Description: Reads from test result file or reruns tests and plots
% results
% Most Recent Change: Travis Hastreiter 24 November, 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Get Test Result Data
test_file_name = "Tests/DelayModel.mldatx";
% Choose to load test results from a file or rerun the test
load_results = true;
result_file_name = "Tests/delaytestresults.mldatx"; 
result_signals = ["error", "x"]; % NOTE: Don't need these but just showing how to get them as an example

% Read data from results file or run the tests and read
[ResultsObj, results, result_arrays] = ReadControllerTestResults(test_file_name,load_results,result_file_name,result_signals);

%% Plot 
% Plot the time it takes to land vs the delay time for each controller
controller_names = string(fieldnames(results));
for r = 1:numel(controller_names)
    scatter(results.(controller_names(r)).optimization_time, results.(controller_names(r)).stop_time, 'LineWidth',2)
    hold on;
end
hold off;
title("Time to Land vs Delay Time by Controller")
subtitle("Missing entries indicate the controller was unable to land with that delay")
xlabel("Delay Time [s]")
ylabel("Time to Land [s]")
legend(controller_names, Location="eastoutside", Interpreter="none")

%% Cleanup
% Delete result object
remove(ResultsObj)