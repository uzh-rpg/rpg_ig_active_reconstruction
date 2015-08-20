% shows the time used to retrieve the infromation scores (only for one
% simulation run though)

clear all;
%close all;

A = importdata('../data/view_planning/youbot_simulation/cost_only/planning_data1427818423_timing.data',' ',1);
B = importdata('../data/view_planning/youbot_simulation/cost_only/planning_data1427818423.data',' ',1);

reconstruction_data = namedstructure(B);
average_cost_retrieval_time_per_view = A.data(:,1);
average_information_retrieval_time_per_view = A.data(:,2);

figure
hold on;


plot(reconstruction_data.totalnrofnodes,average_information_retrieval_time_per_view,'b');


grid on
xlabel('Nr of nodes in octomap.');
ylabel('Average information score retrieval time per view [s]');



figure
hold on;

counter=1:length(reconstruction_data.total_cost);
plot(counter,reconstruction_data.total_cost,'b');


grid on
xlabel('View [nr]');
ylabel('Cost');