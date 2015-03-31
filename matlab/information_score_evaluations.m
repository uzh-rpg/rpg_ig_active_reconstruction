clear all;
close all;

A = importdata('../data/view_planning/flying_stereo_cam_data/information_score_tests/AverageEndPointUncertainty/planning_data1427694255.data',' ',1);
averageendpointuncertainty = namedstructure_old(A);
A = importdata('../data/view_planning/flying_stereo_cam_data/information_score_tests/AverageUncertainty/planning_data1427689366.data',' ',1);
averageuncertainty = namedstructure_old(A);
A = importdata('../data/view_planning/flying_stereo_cam_data/information_score_tests/ClassicFrontier/planning_data1427737352.data',' ',1);
classicfrontier = namedstructure_old(A);
A = importdata('../data/view_planning/flying_stereo_cam_data/information_score_tests/EndNodeOccupancySum/planning_data1427741013.data',' ',1);
endnodeoccupancysum = namedstructure_old(A);
A = importdata('../data/view_planning/flying_stereo_cam_data/information_score_tests/NrOfUnknownVoxels/planning_data1427686103.data',' ',1);
nrofunknownvoxels = namedstructure_old(A);
A = importdata('../data/view_planning/flying_stereo_cam_data/information_score_tests/UnknownObjectSideFrontier/planning_data1427731574.data',' ',1);
unknownobjectsidefrontier = namedstructure_old(A);
A = importdata('../data/view_planning/flying_stereo_cam_data/information_score_tests/UnknownObjectVolumeFrontier/planning_data1427734530.data',' ',1);
unknownobjectvolumefrontier = namedstructure_old(A);

plotnames = {'Average end point uncertainty','Average uncertainty','Classic frontier','End node occupancy sum','Nr of unknown voxels','Unknown objectside frontier','Unknown objectvolume frontier'};

plotfunctions(1) = averageendpointuncertainty;
plotfunctions(2) = averageuncertainty;
plotfunctions(3) = classicfrontier;
plotfunctions(4) = endnodeoccupancysum;
plotfunctions(5) = nrofunknownvoxels;
plotfunctions(6) = unknownobjectsidefrontier;
plotfunctions(7) = unknownobjectvolumefrontier;

plotcolors = ['y','m','c','r','g','b','k'];

counter = 1:1:20;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).averageendpointuncertainty(1:20),plotcolors(i));
end

plot(counter,averageendpointuncertainty.total_score(1:20),'y','LineWidth',2);

legend(plotnames);

grid on
xlabel('Reconstruction step');
ylabel('Average end point uncertainty');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).averageuncertainty(1:20),plotcolors(i));
end

plot(counter,averageuncertainty.total_score(1:20),plotcolors(2),'LineWidth',2);

legend(plotnames);

grid on
xlabel('Reconstruction step');
ylabel('Average uncertainty on rays');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).classicfrontier(1:20),plotcolors(i));
end

plot(counter,classicfrontier.total_score(1:20),plotcolors(3),'LineWidth',2);

legend(plotnames);

grid on
xlabel('Reconstruction step');
ylabel('Classic frontier score');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).endnodeoccupancysum(1:20),plotcolors(i));
end

plot(counter,endnodeoccupancysum.total_score(1:20),plotcolors(4),'LineWidth',2);

legend(plotnames);

grid on
xlabel('Reconstruction step');
ylabel('End node occupancy likelihood sum');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).nrofunknownvoxels(1:20),plotcolors(i));
end

plot(counter,nrofunknownvoxels.total_score(1:20),plotcolors(5),'LineWidth',2);

legend(plotnames);

grid on
xlabel('Reconstruction step');
ylabel('Nr of unknown voxels on ray');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).unknownobjectsidefrontier(1:20),plotcolors(i));
end

plot(counter,unknownobjectsidefrontier.total_score(1:20),plotcolors(6),'LineWidth',2);

legend(plotnames);

grid on
xlabel('Reconstruction step');
ylabel('Unknown objectside frontier score');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).unknownobjectvolumefrontier(1:20),plotcolors(i));
end

plot(counter,unknownobjectvolumefrontier.total_score(1:20),plotcolors(7),'LineWidth',2);

legend(plotnames);

grid on
xlabel('Reconstruction step');
ylabel('Unknown objectvolume frontier score');



% nr of occupieds plot
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).totalnrofoccupieds(1:20),plotcolors(i));
end

legend(plotnames);
grid on
xlabel('Reconstruction step');
ylabel('Nr of occupieds');

% totaloccupancycertainty plot
figure
hold on;

for i=1:length(plotfunctions)
    plot(counter,plotfunctions(i).totaloccupancycertainty(1:20),plotcolors(i));
end

legend(plotnames);
grid on
xlabel('Reconstruction step');
ylabel('Sum over the occupancy likelihood of all occupieds');