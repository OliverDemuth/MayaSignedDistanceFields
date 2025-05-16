% XROMM_measurements.m
%   This script quantifies translational and rotational degrees of 
%   freedom in XROMM data and calculates the mean joint centre using
%   helical axes.
%
%   Written by Oliver Demuth based on modified scripts from Hazel Richards
%
%   Last updated 16.05.2025 - Oliver Demuth


clear;
clc;

%% Load data

%===== RLP2 =====%

RLP2_BM = 0.527; % set body mass in kg
RLP2_BM_norm = RLP2_BM^0.33;
RLP2_HAF = 6.48; % coracoid glenoid facet length/humeral articulation facet length
RLP2_HC = 14.06; % minimum humeral circumference

RLP2_Norm = RLP2_HC; % set normalisation

% load data
RLP2_Data1 = csvread('RLP2_Trial_0001_shoulder_ROM.csv',1,0);
RLP2_Data1 = fillmissing(RLP2_Data1,'makima',1);
RLP2_Data2 = csvread('RLP2_Trial_0002_shoulder_ROM.csv',1,0);
RLP2_Data2 = fillmissing(RLP2_Data2,'makima',1);
% RLP2_Data3 = csvread('RLP2_Trial_0003_shoulder_ROM.csv',1,0);
% RLP2_Data3 = fillmissing(RLP2_Data3,'makima',1);
% RLP2_Data4 = csvread('RLP2_Trial_0004_shoulder_ROM.csv',1,0);
% RLP2_Data4 = fillmissing(RLP2_Data4,'makima',1);
% RLP2_Data5 = csvread('RLP2_Trial_0005_shoulder_ROM.csv',1,0);
% RLP2_Data5 = fillmissing(RLP2_Data5,'makima',1);
% RLP2_Data6 = csvread('RLP2_Trial_0006_shoulder_ROM.csv',1,0);
% RLP2_Data6 = fillmissing(RLP2_Data6,'makima',1);

RLP2_Data = vertcat(RLP2_Data1, RLP2_Data2); %, RLP2_Data3, RLP2_Data4, RLP2_Data5, RLP2_Data6);

RLP2_Data_Norm = RLP2_Data(:,7:12);
RLP2_Data_Norm(:,1:3) = RLP2_Data_Norm(:,1:3)/RLP2_Norm;
RLP2_XROMM = RLP2_Data_Norm(:,4:6);

%writematrix(RLP2_XROMM,'RLP2_XROMM_rot.csv');
%writematrix(RLP2_Data_Norm(:,1:3),'RLP2_XROMM_trans.csv');

RLP2_lig_Data = importDataset('RLP2_new_lig_Trial_000',1,6);
RLP2_ROM_lig = importDataset('RLP2_ROM_new_lig_part_',1,17);

%===== RLP3 =====%

RLP3_BM = 0.492; % set body mass in kg
RLP3_BM_norm = RLP3_BM^0.33;
RLP3_HAF = 6.29; % coracoid glenoid facet length/humeral articulation facet length
RLP3_HC = 14.83; % minimum humeral circumference

RLP3_Norm = RLP3_HC;  % set normalisation

% load data
RLP3_Data1 = csvread('RLP3_Trial_0001_shoulder_ROM.csv',1,0);
RLP3_Data2 = csvread('RLP3_Trial_0002_shoulder_ROM.csv',1,0);
RLP3_Data3 = csvread('RLP3_Trial_0003_shoulder_ROM.csv',1,0);
RLP3_Data4 = csvread('RLP3_Trial_0004_shoulder_ROM.csv',1,0);
RLP3_Data5 = csvread('RLP3_Trial_0005_shoulder_ROM.csv',1,0);
RLP3_Data6 = csvread('RLP3_Trial_0006_shoulder_ROM.csv',1,0);
RLP3_Data7 = csvread('RLP3_Trial_0007_shoulder_ROM.csv',1,0);

RLP3_Data = vertcat(RLP3_Data1, RLP3_Data2, RLP3_Data3, RLP3_Data4, RLP3_Data5, RLP3_Data6, RLP3_Data7);
RLP3_Data_Norm = RLP3_Data(:,7:12);
RLP3_Data_Norm(:,1:3) = RLP3_Data_Norm(:,1:3)/RLP3_Norm;
RLP3_XROMM = RLP3_Data_Norm(:,4:6);

%writematrix(RLP3_XROMM,'RLP3_XROMM_rot.csv');
%writematrix(RLP3_Data_Norm(:,1:3),'RLP3_XROMM_trans.csv');

RLP3_ROM = csvread('RLP3_ROM_simulation.csv',1,0);
%RLP3_XROMM = csvread('RLP3_shoulder_ROM_XYZ.csv');

RLP3_lig_Data = importDataset('RLP3_new_lig_Trial_000',1,7);
RLP3_ROM_lig = importDataset('RLP3_ROM_new_lig_part_',1,17);

%===== RLP4 =====%

RLP4_BM = 0.420; % set body mass in kg
RLP4_BM_norm = RLP4_BM^0.33;
RLP4_HAF = 5.91; % coracoid glenoid facet length/humeral articulation facet length
RLP4_HC = 13.49; % minimum humeral circumference

RLP4_Norm = RLP4_HC;  % set normalisation

% load data
RLP4_Data1 = csvread('RLP4_Trial_0001_shoulder_ROM.csv',1,0);
RLP4_Data2 = csvread('RLP4_Trial_0002_shoulder_ROM.csv',1,0);

RLP4_Data = vertcat(RLP4_Data1, RLP4_Data2); %,, RLP4_Data4, RLP4_Data5, RLP4_Data6);
RLP4_Data_Norm = RLP4_Data(:,7:12);
RLP4_Data_Norm(:,1:3) = RLP4_Data_Norm(:,1:3)/RLP4_Norm;
RLP4_XROMM = RLP4_Data_Norm(:,4:6);

%writematrix(RLP4_XROMM,'RLP4_XROMM_rot.csv');
%writematrix(RLP4_Data_Norm(:,1:3),'RLP4_XROMM_trans.csv');

RLP4_lig_Data = importDataset('RLP4_new_lig_Trial_000',1,2);
RLP3_ROM_lig = importDataset('RLP4_ROM_new_lig_part_',1,17);

%%
RLP2_XROMM = csvread('RLP2_XROMM_rot.csv',0,0);
RLP2_XROMM_trans = csvread('RLP2_XROMM_trans.csv',0,0);
RLP3_XROMM = csvread('RLP3_XROMM_rot.csv',0,0);
RLP3_XROMM_trans = csvread('RLP3_XROMM_trans.csv',0,0);
RLP4_XROMM = csvread('RLP4_XROMM_rot.csv',0,0);
RLP4_XROMM_trans = csvread('RLP4_XROMM_trans.csv',0,0);

RLPs_XROMM = vertcat(RLP2_XROMM,RLP3_XROMM,RLP4_XROMM);

Offset = Maya2TransM([90,0,90]);
[RLP2_mobility,RLP2_shp,RLP2_map] =  calcMobility(RLP2_XROMM,'sin',25,false,Offset);
[RLP3_mobility,RLP3_shp,RLP3_map] =  calcMobility(RLP3_XROMM,'sin',25,false,Offset);
[RLP4_mobility,RLP4_shp,RLP4_map] =  calcMobility(RLP4_XROMM,'sin',25,false,Offset);

RLP_data = vertcat(RLP2_XROMM,RLP3_XROMM,RLP4_XROMM);
Data_Norm = vertcat(RLP2_Data_Norm, RLP3_Data_Norm, RLP4_Data_Norm);

[RLPs_mobility,RLPs_shp,RLPs_map] =  calcMobility(RLP_data,'sin',25,false,Offset);
[RLP3_ROM_mobility,RLP3_ROM_shp,RLP3_ROM_map] =  calcMobility(RLP3_ROM,'sin',25,false,Offset);

%% colours

% define colors

%RLP2_color_hex = ['#821A1A';'#993C2C';'#AF5F3E';'#C68150';'#D9A46F';'#E2C6A2'];
RLP2_color_hex = ['#993C2C';'#D9A46F'];
RLP2_box_hex = '#B0191D';
RLP2_colormap = hex2rgb(RLP2_color_hex);
RLP2_box_color = hex2rgb(RLP2_box_hex);

RLP2_colors1 = repmat(RLP2_colormap(1,:),length(RLP2_Data1),1);
RLP2_colors2 = repmat(RLP2_colormap(2,:),length(RLP2_Data2),1);
% RLP2_colors3 = repmat(RLP2_colormap(3,:),length(RLP2_Data3),1);
% RLP2_colors4 = repmat(RLP2_colormap(4,:),length(RLP2_Data4),1);
% RLP2_colors5 = repmat(RLP2_colormap(5,:),length(RLP2_Data5),1);
% RLP2_colors6 = repmat(RLP2_colormap(6,:),length(RLP2_Data6),1);
RLP2_colors = vertcat(RLP2_colors1, RLP2_colors2); %, RLP2_colors3, RLP2_colors4, RLP2_colors5, RLP2_colors6);


% define colors

RLP3_color_hex = ['#22371A';'#315C28';'#477D35';'#6AA14D';'#9AC076';'#C5DBAB';'#EBF2E0'];
RLP3_box_hex = '#40752D';
RLP3_colormap = hex2rgb(RLP3_color_hex);
RLP3_box_color = hex2rgb(RLP3_box_hex);

RLP3_colors1 = repmat(RLP3_colormap(1,:),length(RLP3_Data1),1);
RLP3_colors2 = repmat(RLP3_colormap(2,:),length(RLP3_Data2),1);
RLP3_colors3 = repmat(RLP3_colormap(3,:),length(RLP3_Data3),1);
RLP3_colors4 = repmat(RLP3_colormap(4,:),length(RLP3_Data4),1);
RLP3_colors5 = repmat(RLP3_colormap(5,:),length(RLP3_Data5),1);
RLP3_colors6 = repmat(RLP3_colormap(6,:),length(RLP3_Data6),1);
RLP3_colors7 = repmat(RLP3_colormap(7,:),length(RLP3_Data7),1);
RLP3_colors = vertcat(RLP3_colors1, RLP3_colors2, RLP3_colors3, RLP3_colors4, RLP3_colors5, RLP3_colors6, RLP3_colors7);


% define colours

RLP4_color_hex = ['#342C7D';'#9E9FC4'];
RLP4_box_hex = '#576FB6';
RLP4_colormap = hex2rgb(RLP4_color_hex);
RLP4_box_color = hex2rgb(RLP4_box_hex);

RLP4_colors1 = repmat(RLP4_colormap(1,:),length(RLP4_Data1),1);
RLP4_colors2 = repmat(RLP4_colormap(2,:),length(RLP4_Data2),1);
RLP4_colors = vertcat(RLP4_colors1, RLP4_colors2);


Data_colors = vertcat(RLP2_colors,RLP3_colors,RLP4_colors);
Data_colormap = vertcat(RLP2_colormap,RLP3_colormap,RLP4_colormap);
Box_colormap = vertcat(RLP2_box_color,RLP3_box_color,RLP4_box_color);

RLP_pc = pointCloud(RLPs_map);


% define labels
RLP2_Label1 = 'RLP2_Trial_1' + strings(length(RLP2_Data1),1);
RLP2_Label2 = 'RLP2_Trial_2' + strings(length(RLP2_Data2),1);

RLP2_labels = vertcat(RLP2_Label1,RLP2_Label2);
RLP2_box_label = 'RLP2' + strings(length(RLP2_labels),1);

RLP3_Label1 = 'RLP3_Trial_1' + strings(length(RLP3_Data1),1);
RLP3_Label2 = 'RLP3_Trial_2' + strings(length(RLP3_Data2),1);
RLP3_Label3 = 'RLP3_Trial_3' + strings(length(RLP3_Data3),1);
RLP3_Label4 = 'RLP3_Trial_4' + strings(length(RLP3_Data4),1);
RLP3_Label5 = 'RLP3_Trial_5' + strings(length(RLP3_Data5),1);
RLP3_Label6 = 'RLP3_Trial_6' + strings(length(RLP3_Data6),1);
RLP3_Label7 = 'RLP3_Trial_7' + strings(length(RLP3_Data7),1);

RLP3_labels = vertcat(RLP3_Label1,RLP3_Label2,RLP3_Label3,RLP3_Label4,RLP3_Label5,RLP3_Label6,RLP3_Label7);
RLP3_box_label = 'RLP3' + strings(length(RLP3_labels),1);

RLP4_Label1 = 'RLP4_Trial_1' + strings(length(RLP4_Data1),1);
RLP4_Label2 = 'RLP4_Trial_2' + strings(length(RLP4_Data2),1);

RLP4_labels = vertcat(RLP4_Label1,RLP4_Label2);
RLP4_box_label = 'RLP4' + strings(length(RLP4_labels),1);

Data_labels = vertcat(RLP2_labels, RLP3_labels, RLP4_labels);
Box_labels = vertcat(RLP2_box_label, RLP3_box_label, RLP4_box_label);

%%
RLPs_fig = figure;

hold on;
pcshow(pointCloud(RLP2_map).Location,RLP2_colors);
hold on;
pcshow(pointCloud(RLP3_map).Location,RLP3_colors);
hold on;
pcshow(pointCloud(RLP4_map).Location,RLP4_colors);

set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

% legend('RLP2','RLP3','RLP4');
xlimits = xlim;
ylimits = ylim;
zlimits = zlim;

interval = 30;

xMin = floor(xlimits(1)/interval/2)*interval*2;
xMax = ceil(xlimits(2)/interval/2)*interval*2;
yMin = floor(ylimits(1)/interval/2)*interval*2;
yMax = ceil(ylimits(2)/interval/2)*interval*2;
zMin = floor(zlimits(1)/interval)*interval;
zMax = ceil(zlimits(2)/interval)*interval;

xl = xMin:interval:xMax;
yl = yMin:interval:yMax;
zl = zMin:interval:zMax;

hold on

patch([xMin xMin xMax xMax],...
      [yMin yMax yMax yMin],...
      [zMin zMin zMin zMin],"white","LineStyle","none","FaceAlpha",1);

plot_euler_lines(xl,yl,zMin,[0.85 0.85 0.85],0.5,'sin');

hold on

scatter3(RLP2_map(:,1),RLP2_map(:,2),repmat(zMin,[length(RLP2_map),1]),1,RLP2_colors,'filled','MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.1);
scatter3(RLP3_map(:,1),RLP3_map(:,2),repmat(zMin,[length(RLP3_map),1]),1,RLP3_colors,'filled','MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.1);
scatter3(RLP4_map(:,1),RLP4_map(:,2),repmat(zMin,[length(RLP4_map),1]),1,RLP4_colors,'filled','MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.1);

xlim([xMin xMax])
ylim([yMin yMax])
zlim([zMin zMax])

hold off

xticks(xl)
yticks(yl)
zticks(zl)

xlabel('FE_s_c');
ylabel('ABAD');
zlabel('LAR');

view (45,25);

grid on;
axis on;


% plot figure for RLP2

RLP2_fig = figure;

hold on;
pcshow(pointCloud(RLP2_map).Location,RLP2_colors);

set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

hold on

Shape2 = plot(RLP2_shp);
set(Shape2, 'facecolor',RLP2_box_color,'edgecolor',RLP2_box_color, 'facealpha', '0.05', 'edgealpha', '0.05'); %set colour and opacity of alpha shape

hold on

patch([xMin xMin xMax xMax],...
      [yMin yMax yMax yMin],...
      [zMin zMin zMin zMin],"white","LineStyle","none","FaceAlpha",1);

plot_euler_lines(xMin:interval*2:xMax,yMin:interval*2:yMax,zMin,[0.85 0.85 0.85],0.5,'sin');

hold on

scatter3(RLP2_map(:,1),RLP2_map(:,2),repmat(zMin,[length(RLP2_map),1]),1,RLP2_colors,'filled','MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.1);

xlim([xMin xMax])
ylim([yMin yMax])
zlim([zMin zMax])

hold off

xticks(xMin:interval*2:xMax)
yticks(yMin:interval*2:yMax)
zticks(zMin:interval*2:zMax)

xlabel('FE_s_c');
ylabel('ABAD');
zlabel('LAR');

view (45,25);

grid on;
axis on;


% plot figure for RLP3

RLP3_fig = figure;

hold on;
pcshow(pointCloud(RLP3_map).Location,RLP3_colors);

set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

hold on
Shape3 = plot(RLP3_shp);
set(Shape3, 'facecolor',RLP3_box_color,'edgecolor',RLP3_box_color, 'facealpha', '0.05', 'edgealpha', '0.05'); %set colour and opacity of alpha shape

hold on

patch([xMin xMin xMax xMax],...
      [yMin yMax yMax yMin],...
      [zMin zMin zMin zMin],"white","LineStyle","none","FaceAlpha",1);

hold on

scatter3(RLP3_map(:,1),RLP3_map(:,2),repmat(zMin,[length(RLP3_map),1]),1,RLP3_colors,'filled','MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.1);


plot_euler_lines(xMin:interval*2:xMax,yMin:interval*2:yMax,zMin,[0.85 0.85 0.85],0.5,'sin');

xlim([xMin xMax])
ylim([yMin yMax])
zlim([zMin zMax])

hold off

xticks(xMin:interval*2:xMax)
yticks(yMin:interval*2:yMax)
zticks(zMin:interval*2:zMax)

xlabel('FE_s_c');
ylabel('ABAD');
zlabel('LAR');

view (45,25);

grid on;
axis on;


% plot figure for RLP4

RLP4_fig = figure;

hold on;
pcshow(pointCloud(RLP4_map).Location,RLP4_colors);

set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

hold on
Shape4 = plot(RLP4_shp);
set(Shape4, 'facecolor',RLP4_box_color,'edgecolor',RLP4_box_color, 'facealpha', '0.05', 'edgealpha', '0.05'); %set colour and opacity of alpha shape

hold on

patch([xMin xMin xMax xMax],...
      [yMin yMax yMax yMin],...
      [zMin zMin zMin zMin],"white","LineStyle","none","FaceAlpha",1);

hold on

scatter3(RLP4_map(:,1),RLP4_map(:,2),repmat(zMin,[length(RLP4_map),1]),1,RLP4_colors,'filled','MarkerEdgeAlpha', 0.1, 'MarkerFaceAlpha', 0.1);


plot_euler_lines(xMin:interval*2:xMax,yMin:interval*2:yMax,zMin,[0.85 0.85 0.85],0.5,'sin');

xlim([xMin xMax])
ylim([yMin yMax])
zlim([zMin zMax])

hold off

xticks(xMin:interval*2:xMax)
yticks(yMin:interval*2:yMax)
zticks(zMin:interval*2:zMax)

xlabel('FE_s_c');
ylabel('ABAD');
zlabel('LAR');

view (45,25);

grid on;
axis on;

%% get Maya angles for some exemplary poses

mayaRot = map2Maya([2.8098 ...
                    33.6048, ...
                    22.4264],Offset)

mayaRot = map2Maya([38.7126, ...
                    127.1487, ...
                    -34.8602],Offset)

mayaRot = map2Maya([36.0951, ...
                    49.2737, ...
                    -48.9555],Offset)

mayaRot = map2Maya([61.924, ...
                    100.8336, ...
                    -100.6798],Offset)

mayaRot = map2Maya([78.0161, ...
                    118.5311, ...
                    -56.479],Offset)


%% export figures

% exportgraphics(RLPs_fig,'Plots/RLPs_XROMM_plot.pdf','ContentType','vector');
% exportgraphics(RLP2_fig,'Plots/RLP2_XROMM_plot.pdf','ContentType','vector');
% exportgraphics(RLP3_fig,'Plots/RLP3_XROMM_plot.pdf','ContentType','vector');
% exportgraphics(RLP4_fig,'Plots/RLP4_XROMM_plot.pdf','ContentType','vector');

%% offset figure

normOffset = vecnorm(transpose(Data_Norm(:,1:3)));

figure
hold on
scatter(1:length(normOffset),normOffset,[],Data_colors,'.');
hold off

xlabel('Frames');
ylabel('Normalized Translational Offset [d_T/HAF]');


%% For each frame formulate the 4×4 transformation matrix (Z-Y'-X" Euler 

DataSubset = Data_Norm;

tX = DataSubset(:,1);
tY = DataSubset(:,2);
tZ = DataSubset(:,3);
rX = DataSubset(:,4);
rY = DataSubset(:,5);
rZ = DataSubset(:,6);


numFrames = length(DataSubset);

% intrinsic convention, corresponding to FE, then ABAD, then LAR)
Coordinates = [];

%Create 3 unit vectors in 3D space (any three are necessary and sufficient 
%to describe 3D rigid body motion) - these are fixed with respect to the
%ulna in the neutral position. Here we will use the basis vectors.
P0 = [eye(3); 1 1 1];

for i = 1:numFrames
    %Construct transformation matrix

    rot = eul2rotm([rZ(i) rY(i) rX(i)]);
    trans = [tX(i);tY(i);tZ(i)];
    rottrans = horzcat(rot,trans);
    T = vertcat(rottrans,[0 0 0 1]);

    %Multiply by unit vectors
    P1 = T*P0;
    Coordinates = vertcat(Coordinates, horzcat(P1(1:3,1)',P1(1:3,2)',P1(1:3,3)'));
end

%Calculate 4×4 transformation matrix from one pose to the next
Transforms = zeros(4,4,numFrames-1);
for i = 1:numFrames-1
    Transforms(:,:,i) = soder(Coordinates(i:i+1,:));
end

% Calculate finite helical axis for movement from one pose to the next
FHA_n = zeros(numFrames-1,3);
FHA_p = zeros(numFrames-1,3);
FHA_phi = zeros(numFrames-1,1);
FHA_d = zeros(numFrames-1,1);
for i = 1:numFrames-1
    [n,p,phi,d] = screw(Transforms(:,:,i));
    FHA_n(i,:) = n;
    FHA_p(i,:) = p;
    FHA_phi(i) = phi;
    FHA_d(i) = d;
end

%Calculate cumulative rotation and translation across each pose
Phi_cum = cumsum(FHA_phi);
d_cum = cumsum(FHA_d);

% Calculate mean helical axis and mean joint centre

%Mean helical axis is taken to be the directional mean using circular
%statistics
Mean_helical_axis = sum(FHA_n)/(numFrames-1);
Mean_helical_axis = Mean_helical_axis/norm(Mean_helical_axis);

%Mean joint centre is taken as the point that is closest to all finite
%helical axes in the least squares sense (i.e., minimizes the sum of
%squared distances to each line).
FiniteAxesStart = FHA_p-75*FHA_n;
FiniteAxesEnd = FHA_p+75*FHA_n;
[Mean_joint_centre,Centre_Error] = lineIntersect3D(FiniteAxesStart,FiniteAxesEnd);
Mean_joint_error = mean(Centre_Error);
Mean_joint_std = std(Centre_Error);

%Calculate helix parameters
%radius is hard-coded here, taken as the radius of the sphere fitted to the
%capitulum in Blender
[pitch, helix_angle] = helix(27.436, max(d_cum)-min(d_cum), max(Phi_cum)-min(Phi_cum));

% Calculate offset from mean joint centre

TransOffset = horzcat(tX,tY,tZ);
CorrectedOffset = TransOffset -  Mean_joint_centre;
CorrectedOffset1 = vertcat(CorrectedOffset,[0 0 0]);

% Create convex hull of data points

[k1,av1] = convhull(CorrectedOffset);

% Plot offset vs joint angles

CdataZ = rZ; %FE
CdataY = rY; %ABAD
CdataX = rX; %LAR

% set colormap
m = 360;
cmap = brewermap(m,'*RdYlBu');

% make it into a index image.
cmin = -180;
cmax = 180;

indexZ = fix((CdataZ-cmin)/(cmax-cmin)*m)+1; %A
indexY = fix((CdataY-cmin)/(cmax-cmin)*m)+1; %A
indexX = fix((CdataX-cmin)/(cmax-cmin)*m)+1; %A
% Then to RGB
RGBdataZ = ind2rgb(indexZ,cmap);
RGBZ = horzcat((RGBdataZ(:,:,1)),(RGBdataZ(:,:,2)),(RGBdataZ(:,:,3)));
RGBdataY = ind2rgb(indexY,cmap);
RGBY = horzcat((RGBdataY(:,:,1)),(RGBdataY(:,:,2)),(RGBdataY(:,:,3)));
RGBdataX = ind2rgb(indexX,cmap);
RGBX = horzcat((RGBdataX(:,:,1)),(RGBdataX(:,:,2)),(RGBdataX(:,:,3)));

% plot figure

fig = figure;
subplot(2,7,[1,2]);
scatter(CorrectedOffset(:,1),CorrectedOffset(:,2),10,RGBZ,'Marker','.');
title('Flexion/Extension')
ylabel({'Lateral - Medial';'Normalised Offset'})
set(gca,'XAxisLocation','top')
set(gca, 'FontName', 'Helvetica')
grid on;

subplot(2,7,[3,4]);
scatter(CorrectedOffset(:,1),CorrectedOffset(:,2),10,RGBY,'Marker','.');
title('Abduction/Adduction')
set(gca,'XAxisLocation','top');
set(gca,'YTickLabel',[]);
set(gca, 'FontName', 'Helvetica')
grid on;

subplot(2,7,[5,6]);
scatter(CorrectedOffset(:,1),CorrectedOffset(:,2),10,RGBX,'Marker','.');
title('Long-axis Rotation')
set(gca,'XAxisLocation','top');
set(gca,'YAxisLocation','right');
set(gca, 'FontName', 'Helvetica')
grid on;

subplot(2,7,[8,9]);
scatter(CorrectedOffset(:,1),CorrectedOffset(:,3),10,RGBZ,'Marker','.');
xlabel({'Cranial - Caudal';'Normalised Offset'})
ylabel({'Ventral - dorsal';'Normalised Offset'})
set(gca, 'FontName', 'Helvetica')
grid on;

subplot(2,7,[10,11]);
scatter(CorrectedOffset(:,1),CorrectedOffset(:,3),10,RGBY,'Marker','.');
xlabel({'Cranial - Caudal';'Normalised Offset'})
set(gca,'YTickLabel',[]);
set(gca, 'FontName', 'Helvetica')
grid on;

subplot(2,7,[12,13]);
scatter(CorrectedOffset(:,1),CorrectedOffset(:,3),10,RGBX,'Marker','.');
xlabel({'Cranial - Caudal';'Normalised Offset'})
set(gca,'YAxisLocation','right');
set(gca, 'FontName', 'Helvetica')
grid on;

subplot(2,7,[7,14])
set(gca, 'FontName', 'Helvetica')
axis off;
clim([cmin cmax]);
cb = colorbar;
colormap(cmap);
cb.Label.String  = 'Deviation [°]';

% export figure
%exportgraphics(fig,'RLP_trans_vs_rot_offset.pdf','ContentType','vector');


%%

RLPs_plot_map = RLPs_map;

plotInterval = 60;
minRot = floor(min(RLPs_plot_map)/plotInterval)*plotInterval;
maxRot = ceil(max(RLPs_plot_map)/plotInterval)*plotInterval;

dataDir = ["cranial(-) caudal(+)";
           "lateral(-) medial(+)";
           "ventral(-) dorsal(+)"];

plotData = Data_Norm(:,1:3);

rotChar = ['X','Y','Z'];

for i = 1:3
    fig = figure();

    cmax = max(abs(plotData),[],'all');
    colormap(brewermap(256,'*PuOr'))

    alphaValues = 0.9/cmax*abs(plotData(:,i))+0.1;

    % plot data

    scatter3(RLPs_plot_map(:,1),RLPs_plot_map(:,2),RLPs_plot_map(:,3),1,plotData(:,i),'.')
    %alpha(alphaValues)
    hold on;

    scatter3(RLPs_plot_map(:,1),RLPs_plot_map(:,2),repmat(minRot(3),[length(RLPs_plot_map),1]),1,plotData(:,i),'filled','MarkerEdgeAlpha', 0.15, 'MarkerFaceAlpha', 0.15);
    scatter3(repmat(minRot(1),[length(RLPs_plot_map),1]),RLPs_plot_map(:,2),RLPs_plot_map(:,3),1,plotData(:,i),'filled','MarkerEdgeAlpha', 0.15, 'MarkerFaceAlpha', 0.15);
    scatter3(RLPs_plot_map(:,1),repmat(maxRot(2),[length(RLPs_plot_map),1]),RLPs_plot_map(:,3),1,plotData(:,i),'filled','MarkerEdgeAlpha', 0.15, 'MarkerFaceAlpha', 0.15);

    xl = minRot(1):plotInterval:maxRot(1);
    yl = minRot(2):plotInterval:maxRot(2);
    zl = minRot(3):plotInterval:maxRot(3);
    
    hold on
    
    patch([minRot(1) minRot(1) maxRot(1) maxRot(1)],...
          [minRot(2) maxRot(2) maxRot(2) minRot(2)],...
          [minRot(3) minRot(3) minRot(3) minRot(3)],"white","LineStyle","none","FaceAlpha",1);

    plot_euler_lines(xl,yl,minRot(3),[0.85 0.85 0.85],0.5,'sin');

    xlim([minRot(1) maxRot(1)])
    ylim([minRot(2) maxRot(2)])
    zlim([minRot(3) maxRot(3)])

    hold off

    xticks(xl)
    yticks(yl)
    zticks(zl)

    xlabel('FE_s_c');
    ylabel('ABAD');
    zlabel('LAR');

    view (45,25);

    grid on;
    axis on;

    clim([-cmax cmax]);
    
    cbar = colorbar;
    cbar.Label.String = append('Norm. ', char(dataDir(i)),' offset [d_T/HC]');

    ax = gca;
    ax.FontSize = 15;

    %exportgraphics(fig,append('Plots/RLPs_t',rotChar(i),'.pdf'),'ContentType','vector');

end

% get extreme translations from data

[minT,minTidx] = min(plotData);
[maxT,maxTidx] = max(plotData);

TDataIdcs = horzcat(minTidx,maxTidx);

floor(TDataIdcs/5458); % get trial
mod(TDataIdcs,5458); % get frame in respective trial

TData = Data_Norm(horzcat(minTidx,maxTidx),:);
TData(:,1:3) = TData(:,1:3)*RLP3_HC;

%% Fit ellipsoid to the translation offset

% run PCA on translation offset points to get principal component axes

[coeff,score,latent] = pca(CorrectedOffset);
[coeff1,score1,latent1] = pca(CorrectedOffset1); 

PCAvar = latent/sum(latent)*100;

MeanJointPCA = score1(length(score1),:);

% get percentiles of translations along PC axes

Centroid = mean(CorrectedOffset);

p=100; % set according to Confidence Interval, e.g. 95

CIx = prctile(score(:,1), [(100-p)/2, p+(100-p)/2]); % x CI [left, right]
CIy = prctile(score(:,2), [(100-p)/2, p+(100-p)/2]); % x CI [left, right]
CIz = prctile(score(:,3), [(100-p)/2, p+(100-p)/2]); % x CI [left, right]

% calculate CI range and their centre

CIrng(1) = CIx(2)-CIx(1); % CI range (x)
CIrng(2) = CIy(2)-CIy(1); % CI range (y)
CIrng(3) = CIz(2)-CIz(1); % CI range (z)

CI_centre =  [mean(CIx) mean(CIy) mean(CIz)];

% calculate centre position of ellipsoid

CIcentrePos = CI_centre * coeff';
CIcentrePos = bsxfun(@plus, CIcentrePos, Centroid);

% calculate euler angles from rotation matrix of PCA

Rotation = rad2deg(rotm2eul(coeff));

% calculate all Maya attributes

MayaTranslation = [CIcentrePos(1,1),CIcentrePos(1,2),CIcentrePos(1,3)];
MayaRotation = [Rotation(3),Rotation(2),Rotation(1)];
MayaScale = CIrng/2; % ellipsoid radii

% export

RLP2_JCP = Mean_joint_centre*(RLP2_Norm);
RLP2_Trans = MayaTranslation*(RLP2_Norm);
RLP2_Rot = MayaRotation;
RLP2_Scale = MayaScale*(RLP2_Norm);

RLP2_Ellipsoid_Attr = horzcat(RLP2_JCP,RLP2_Trans,RLP2_Rot,RLP2_Scale,RLP2_Norm);

RLP3_JCP = Mean_joint_centre*(RLP3_Norm);
RLP3_Trans = MayaTranslation*(RLP3_Norm);
RLP3_Rot = MayaRotation;
RLP3_Scale = MayaScale*(RLP3_Norm);

RLP3_Ellipsoid_Attr = horzcat(RLP3_JCP,RLP3_Trans,RLP3_Rot,RLP3_Scale,RLP3_Norm);

RLP4_JCP = Mean_joint_centre*(RLP4_Norm);
RLP4_Trans = MayaTranslation*(RLP4_Norm);
RLP4_Rot = MayaRotation;
RLP4_Scale = MayaScale*(RLP4_Norm);

RLP4_Ellipsoid_Attr = horzcat(RLP4_JCP,RLP4_Trans,RLP4_Rot,RLP4_Scale,RLP4_Norm);

Ellipsoid_colnames = {'MJC_PosX','MJC_PosY','MJC_PosZ', ...
    'Ellipsoid_tX','Ellipsoid_tY','Ellipsoid_tZ', ...
    'Ellipsoid_rX','Ellipsoid_rY','Ellipsoid_rZ', ...
    'Ellipsoid_sX','Ellipsoid_sY','Ellipsoid_sZ',...
    'Scale_Factor'};

MayaEllipsoidAttr = array2table(vertcat(RLP2_Ellipsoid_Attr,RLP3_Ellipsoid_Attr,RLP4_Ellipsoid_Attr),'VariableNames',Ellipsoid_colnames);

%writetable(MayaEllipsoidAttr,'Data/RLP_Maya_Attributes.csv');

%% create ellipsoid

sph = extendedObjectMesh('sphere',18);
dims = struct('Length',CIrng(1),'Width',CIrng(2),'Height',CIrng(3), 'OriginOffset', [0,0,0]);
ellipsoid = scaleToFit(sph,dims);

% rotate and translate ellipsoid

ellipsoid = rotate(ellipsoid,coeff');
ellipsoid = translate(ellipsoid,MayaTranslation);

% triangulate ellipsoid 
ConvHull = triangulation(k1,CorrectedOffset(:,1),CorrectedOffset(:,2),CorrectedOffset(:,3));
TRellipsoid = triangulation(ellipsoid.Faces, ellipsoid.Vertices);

% plot PC1 vs PC2 and confidence ellipsoid

variables = {'Caudal Offset','Medial Offset','Dorsal Offset'};

Fig_legend = unique(Data_labels);
Fig_legend = strrep(Fig_legend,'_',' ');
Fig_legend = vertcat(Fig_legend,'Mean Joint Centre');


TransPC12 = figure;
h = scatterhist(score(:,1),score(:,2),'Group',Data_labels,'Marker','.', 'Color', Data_colormap);
hold on;
%biplot(coeff(:,[1,2]),'VarLabels',variables);
hold on;
plot(MeanJointPCA(1),MeanJointPCA(2),'Marker','+','color','k','MarkerSize',10);
hold on;
boxchart(h(2),score(:,1),'Group',Box_labels,'orientation','horizontal','JitterOutliers','on','MarkerStyle','.','MarkerSize',0.5);
colororder(Box_colormap);
boxchart(h(3),score(:,2),'Group',Box_labels,'orientation','horizontal','JitterOutliers','on','MarkerStyle','.','MarkerSize',0.5);
colororder(Box_colormap);
set(h(2:3),'XTickLabel','');
view(h(3),[270,90]);  % Rotate the Y plot
axis(h(1),'auto');  % Sync axes
hold off;
legend(Fig_legend);

xlab = sprintf('%.2f',PCAvar(1));
xlabel(['PC1 (', xlab, '%)']);
ylab = sprintf('%.2f',PCAvar(2));
ylabel(['PC2 (', ylab, '%)']);

% Draw ellipses
llc = [CIx(1), CIy(1)]; % (x,y) lower left corners
rectangle('Position',[llc,CIrng(1,(1:2))],'Curvature',[1,1],'EdgeColor','#595959','LineWidth',1);

% plot PC1 vs PC3 and confidence ellipsoid
TransPC13 = figure;

h = scatterhist(score(:,1),score(:,3),'Group',Data_labels,'Marker','.', 'Color', Data_colormap);
hold on;
%biplot(coeff(:,[1,3]),'VarLabels',variables);
hold on;
plot(MeanJointPCA(1),MeanJointPCA(3),'Marker','+','color','k','MarkerSize',10);
hold on;
boxchart(h(2),score(:,1),'Group',Box_labels,'orientation','horizontal','JitterOutliers','on','MarkerStyle','.','MarkerSize',0.5);
colororder(Box_colormap);
b2 = boxchart(h(3),score(:,3),'Group',Box_labels,'orientation','horizontal','JitterOutliers','on','MarkerStyle','.','MarkerSize',0.5);
colororder(Box_colormap);
set(h(2:3),'XTickLabel','');
view(h(3),[270,90]);  % Rotate the Y plot
axis(h(1),'auto');  % Sync axes
hold off;
legend(Fig_legend);

xlab = sprintf('%.2f',PCAvar(1));
xlabel(['PC1 (', xlab, '%)']);
ylab = sprintf('%.2f',PCAvar(3));
ylabel(['PC3 (', ylab, '%)']);

% Draw ellipses
llc = [CIx(1), CIz(1)]; % (x,y) lower left corners
rectangle('Position',[llc,CIrng(1,([1,3]))],'Curvature',[1,1],'EdgeColor','#595959','LineWidth',1);

%exportgraphics(TransPC12,'Plots/RLP_offset_PC12.pdf','ContentType','vector');
%exportgraphics(TransPC13,'Plots/RLP_offset_PC13.pdf','ContentType','vector');

%% Plot offset data and fitted ellipsoid

% get point cloud data from Offset
ptOffset = pointCloud(CorrectedOffset);

figure
hold on
pcshow(ptOffset.Location, Data_colors);
trisurf(TRellipsoid,'facecolor','#01665e','edgecolor','#01665e', 'facealpha', '0.1', 'edgealpha', '0.1')
colormap(Data_colormap);
set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

hold off

xlabel('Norm. Craniocaudal Offset');
ylabel('Norm. Mediolateral Offset');
zlabel('Norm. Dorsoventral Offset');

grid on;
axis on;



%%
datafig=figure;
tiledlayout(3,3,'TileSpacing','tight','Padding','tight');
nexttile(1,[1 2]);

MJCOffset = (Data_Norm(:,1:3))-Mean_joint_centre;
normMJCOffset = vecnorm(transpose(MJCOffset));

scatter(1:length(normOffset),normOffset,5,Data_colors,'.');
xlabel('Frames');
ylabel({'Norm. Translational'; 'Offset [d_T/HAF]'});

nexttile(4,[2 2]);

pcshow(ptOffset.Location, Data_colors, 'MarkerSize', 10);
hold on
%trisurf(k1,CorrectedOffset(:,1),CorrectedOffset(:,2),CorrectedOffset(:,3),'facecolor','#2b83ba','edgecolor','#2b83ba', 'facealpha', '0.1', 'edgealpha', '0.1')
trisurf(TRellipsoid,'facecolor','#01665e','edgecolor','#01665e', 'facealpha', '0.05', 'edgealpha', '0.05')
colormap(Data_colormap);
set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);set(gca, 'Projection','perspective')

xlabel('Norm. Craniocaudal Offset [d_X/HAF]');
ylabel('Norm. Mediolateral Offset [d_Y/HAF]');
zlabel('Norm. Dorsoventral Offset [d_Z/HAF]');

grid on;
axis on;

%exportgraphics(datafig,'Plots/RLP_offset_plot_new.pdf','ContentType','vector');

%% Cosine correct Data function

function ccData = cc(DataRotation) % Rotation for X, Y and Z axes

first  = DataRotation(:,3); % Z-axis rotation, i.e., FE
second = DataRotation(:,2); % Y-axis rotation, i.e., ABAD
third  = DataRotation(:,1); % X-axis rotation, i.e., LAR

for i = 1:(size(second))
    if (second(i) > 90)
       	first(i) = first(i) - 180;
        second(i)  = 180 - second(i);
        third(i) = third(i) - 180;
    end
end

for i = 1:(size(third))
    if (third(i) > 180)
        third(i) = -1* (360 - third(i)); 
    end
end

correctedFirst = first .* cosd(second);
ccData = horzcat(correctedFirst,second,third);

end

%% Helper functions from the KineMat Toolbox
% Reinschmidt, C. and van den Bogert, T. 1997. KineMat: a MATLAB Toolbox
% for Three-Dimensional Kinematic Analyses. Human Performance Laboratory,
% Calgary.

function [T,res]=soder(data)
% function [T,res]=soder(data)
%
% Description:	Program calculates the transformation matrix T containing
%		the rotation matrix (3x3) and the translation translation
%		vector d (3x1) for a rigid body segment using a singular
%		value decomposition method (Soederkvist & Wedin 1993).
%
% Input:    data:   columns represent the XYZ positions and the rows
%                   represent time.
% Output:   T:      4x4 Matrix containing the rotation matrix R and the
%                   translation d: T = [R,d; 0 0 0 1]
%           res:    norm of residuals (measure of fit; "rigidity" of body
%
% References:     Soderkvist I. and Wedin P. -A., (1993). Determining the
%                 movements of the skeleton using well-configured markers.
%                 Journal of Biomechanics, 26:1473-1477
%
% Author:	Christoph Reinschmidt, HPL, The University of Calgary
%               (Matlab code adapted from Ron Jacobs, 1993)
% Date:		February, 1995
% Last Changes: December 09, 1996
% Version:      3.1

if (size(data,2)/3)~=fix(size(data,2)/3)
    disp('ERROR: input has to be multiple of 3 (XYZ coordinates)'); return
end


A=[reshape(data(1,:)',3,size(data,2)/3)]';
B=[reshape(data(2,:)',3,size(data,2)/3)]';

% Checking for NaNs and also checking if still 3 pts left and if not
% T=[NaN...];
cut=[0];
qA=isnan(A); qB=isnan(B); qAB=[qA,qB];
qsum=sum(qAB'); cut=find(qsum~=0);
A([cut],:)=[]; B([cut],:)=[];
if size(A,1)<3
    T=[NaN,NaN,NaN,NaN;NaN,NaN,NaN,NaN;NaN,NaN,NaN,NaN;NaN,NaN,NaN,NaN;]; return;
end

Amean=mean(A)'; Bmean=mean(B)';

for i=1:size(A,1)-size(cut,2)
    Ai(:,i)=[A(i,:)-Amean']';
    Bi(:,i)=[B(i,:)-Bmean']';
end

C=Bi*Ai';
[P,T,Q]=svd(C);
R=P*diag([1 1 det(P*Q')])*Q';
d=Bmean-R*(Amean);

T=[R,d;0 0 0 1];

% Calculating the norm of residuals
A=A'; A(4,:)=ones(1,size(A,2));
B=B';
Bcalc=T*A; Bcalc(4,:)=[]; Diff=B-Bcalc; Diffsquare=Diff.^2;
%DOF=3*(number of points)-6 unknowns (Hx,Hy,Hz,alpha,beta,gamma):
DOF=size(B,1)*size(B,2)-6;
res=[sum(Diffsquare(:))/DOF].^0.5;
end

function [n,point,phi,t]=screw(T,intersect)
% Calculation of the screw axis
% function [n,point,phi,t]=screw(T)
% Input:    T   matrix containing the rotation matrix and transl. vector
%               [R;[t1,t2,t3]'; 0 0 0 1]
%	    intersect	location of the screw axis where it intersects either the x=0 (intersect=1),
%                       the y=0 (intersect=2), or the z=0 (intersect=3) plane.
%			default: intersect=3
% Output:   n       unit vector with direction of helical axis
%           point   point on helical axis
%           phi     rotation angle (in deg)
%           t       amount of translation along screw axis
%
% Comments:     Note that phi is b/w 0 and 180 deg. Right handed screw
%               axis system. The "sign" of phi can be checked with direction
%               of the unit vector (n).
% References:   (1) Spoor and Veldpaus (1980) Rigid body motion calculated
%                   from spatial co-ordinates of markers.
%                   J Biomech 13: 391-393
%               (2) Berme, Cappozzo, and Meglan. Rigid body mechanics
%                   as applied to human movement studies. In Berme and
%                   Cappozzo: Biomechanics of human movement.
%
% Author:	 Christoph Reinschmidt, HPL, UofCalgary
% Date:          Oct. 03, 1994
% Last Changes:  Nov-20-96

if nargin==1, intersect=[3]; end

R=T(1:3,1:3);

% tmp is matrix in equ. 31 (Spoor and Veldpaus, 1980)
tmp=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];

%calculating n using equ. 31 and 32 (Spoor and Veldpaus, 1980)
n=tmp/norm(tmp);

% calculating phi either with equ. 32 or 34 (Spoor and Veldpaus, 1980)
% depending if sin(phi) smaller or bigger than 0.5*SQRT(2)
if norm(tmp) <= sqrt(2)
    phi=rad2deg(asin(0.5*norm(tmp)));
else  phi=rad2deg(acos(0.5*(R(1,1)+R(2,2)+R(3,3)-1)));
end

%if phi approaches 180 deg it is better to use the following:
%(see Spoor and Veldpaus Eq. 35,36)
if phi>135
    b=[0.5*(R+R')-cos(deg2rad(phi)) * eye(3)];
    b1=[b(:,1)]; b2=[b(:,2)]; b3=[b(:,3)];
    btmp=[b1'*b1;b2'*b2;b3'*b3];
    [bmax,i]=max(btmp);
    n=b(:,i)/sqrt(bmax);
    if sign(R(3,2)-R(2,3)) ~= sign(n(1,1));  n=n.*(-1); end
end

t=n'*T(1:3,4);

% calculate where the screw axis intersects the plane as defined in 'intersect'
Q=R-eye(3);
Q(:,intersect)=-n;
point=Q\[T(1:3,4).*[-1]];
point(intersect,1)=[0];
end

%% Helper function from the MathWorks file exchange (#37192)

function [P_intersect,distances] = lineIntersect3D(PA,PB)
% Find intersection point of lines in 3D space, in the least squares sense.
% PA :          Nx3-matrix containing starting point of N lines
% PB :          Nx3-matrix containing end point of N lines
% P_Intersect : Best intersection point of the N lines, in least squares sense.
% distances   : Distances from intersection point to the input lines
% Anders Eikenes, 2012

Si = PB - PA; %N lines described as vectors
ni = Si ./ (sqrt(sum(Si.^2,2))*ones(1,3)); %Normalize vectors
nx = ni(:,1); ny = ni(:,2); nz = ni(:,3);
SXX = sum(nx.^2-1);
SYY = sum(ny.^2-1);
SZZ = sum(nz.^2-1);
SXY = sum(nx.*ny);
SXZ = sum(nx.*nz);
SYZ = sum(ny.*nz);
S = [SXX SXY SXZ;SXY SYY SYZ;SXZ SYZ SZZ];
CX  = sum(PA(:,1).*(nx.^2-1) + PA(:,2).*(nx.*ny)  + PA(:,3).*(nx.*nz));
CY  = sum(PA(:,1).*(nx.*ny)  + PA(:,2).*(ny.^2-1) + PA(:,3).*(ny.*nz));
CZ  = sum(PA(:,1).*(nx.*nz)  + PA(:,2).*(ny.*nz)  + PA(:,3).*(nz.^2-1));
C   = [CX;CY;CZ];
P_intersect = (S\C)';

if nargout>1
    N = size(PA,1);
    distances=zeros(N,1);
    for i=1:N %This is faster:
        ui=(P_intersect-PA(i,:))*Si(i,:)'/(Si(i,:)*Si(i,:)');
        distances(i)=norm(P_intersect-PA(i,:)-ui*Si(i,:));
    end
end
end


%% HEX to RGB function

function rgb = hex2rgb(hexArray)
    %Converts hexadecimal color code into RGB triplets
    
    rgb = zeros(size(hexArray,1),3);
    for i=1:size(hexArray,1)
        temp_col = hexArray(i,:);
        rgb(i,:) = sscanf(temp_col(2:end),'%2x%2x%2x',[1 3])/255;
    end
end


%% Other custom-built helper function

function [pitch, theta] = helix(radius, translation, rotation)
%Calculates pitch and angle of a helix given the radius, and amounts of
%rotation (degrees) and translation from one point on the helix to another

%Pitch is the amount of translation achieved along the helical axis in one
%full revolution
pitch = translation*(360/rotation);

%The helix angle is calculated by thinking of the helix as being
%'unwrapped' onto the surface of a cylinder; the circumference of that
%cylinder is the 'adjacent' side of the angle, and the pitch (cylinder
%height) is the 'opposite' side of the angle:
theta = atand(pitch/(2*pi*radius));

end

%% Helper functions to streamline calulcations

function [mobility,shp,map] = calcMobility(Data,type,rad,plot,Offset)
    if nargin < 5
        transMat = Maya2TransM(Data);
    else
        transMat = Maya2TransM(Data,Offset);
    end
    
    [plane, abd, axial] = get_spherical_angles_2neu(transMat); % from Lee et al. 2023
    map = euler2map([plane, abd, axial],type); % from Lee et al. 2023
    [mobility,shp] = get_mobility(map,rad,plot); % from Lee et al. 2023
end


% import ligament data

function Data = importDataset(prefix,first,last)
    numRange = first:1:last;
    
    % go through specified ligament files
    for i = 1:length(numRange)
        
        % import data from ligament part csv files
    
        filename = string(prefix) + int2str(numRange(i)) + ".csv";
        
        if i == 1
            Data = csvread(filename,1,0);
        else
            Data = vertcat(Data,csvread(filename,1,0));
        end
    end
end


function rot = map2Maya(map,OffsetMat)
    
    if nargin < 2 
        OffsetMat = eye(4);
    end

    map_2neu = [map(:,1)/sind(map(:,2)),map(:,2),map(:,3)];
    rotMat = get_R_spherical_2neu(map_2neu(:,1),map_2neu(:,2),map_2neu(:,3)); % from Lee et al. 2023
    corrMat = rotMat/OffsetMat;
    rot = flip(rad2deg(rotm2eul(corrMat(1:3,1:3))));
end

