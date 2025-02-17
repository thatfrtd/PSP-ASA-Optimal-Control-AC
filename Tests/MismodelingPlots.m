%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PSP ASA
% Mismodeling Plots Script
% Author: Alex Spada, Travis Hastreiter
% Created On: 2 February 2025
% Description: Reads from test result file or reruns tests and plots
% results
% Most Recent Change: Alex Spada, 2 February 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Get Test Result Data
test_file_name = "Tests/MismodelingModel.mldatx";
% Choose to load test results from a file or rerun the test
load_results = true;
result_file_name = "Tests/MismodelingDataReport.mldatx"; 
result_signals = ["error","x"];

% Read data from results file or run the tests and read
[ResultsObj, results, result_arrays] = ReadControllerTestResults(test_file_name,load_results,result_file_name,result_signals);

%% Plot 
% Plot the time it takes to land vs the delay time for each controller
controller_names = string(fieldnames(results));
for r = 1:numel(controller_names)
    scatter(results.(controller_names(r)).com_error, results.(controller_names(r)).stop_time, 'o','LineWidth',2)
    hold on;
    scatter(results.(controller_names(r)).inertia_error, results.(controller_names(r)).stop_time, "x", "LineWidth", 2)
    hold on;
    scatter(results.(controller_names(r)).mass_error, results.(controller_names(r)).stop_time, "blue",'s', "LineWidth", 2)
    hold on;
    scatter(results.(controller_names(r)).max_gimbal_rate, results.(controller_names(r)).stop_time, "green", "LineWidth", 2)
    hold on;
end

hold off;
title("Time to Land vs Errors By Controller")
subtitle("Missing entries indicate the controller was unable to land with that delay")
xlabel("Errors [%]")
ylabel("Time to Land [s]")
legend("All Errors", Location="eastoutside", Interpreter="none")

%% Cleanup
% Delete result object
remove(ResultsObj)