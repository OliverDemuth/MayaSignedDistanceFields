% Ligament_XROMM_filter.m
%   This script filters XROMM ligament simulation data and removes outliers
%   a 3DOF output and compares the results with XROMM data. Ligament lengths  
%   are filtered and outlieres removed. 6DOF osteological ROM is then 
%   constrained using ligament thersholds and joint poses exceeding the 
%   threshold removed. 
%
%   Written by Oliver Demuth 
%
%   Last updated 07.09.2023 - Oliver Demuth

clear;
clc;

%% Load XROMM Data

Trial_0001 = csvread('RLP3_Trial_0001.csv',1,0);
Trial_0002 = csvread('RLP3_Trial_0002.csv',1,0);
Trial_0003 = csvread('RLP3_Trial_0003.csv',1,0);
Trial_0004 = csvread('RLP3_Trial_0004.csv',1,0);
Trial_0005 = csvread('RLP3_Trial_0005.csv',1,0);
Trial_0006 = csvread('RLP3_Trial_0006.csv',1,0);
Trial_0007 = csvread('RLP3_Trial_0007.csv',1,0);

RawData = vertcat(Trial_0001,Trial_0002,Trial_0003,Trial_0004,Trial_0005,Trial_0006,Trial_0007);

% filter parameters

span = 8;
detect = 'movmedian';
filter = 'rlowess';

% remove outliers and smooth data

Trial_0001_s = ligFilter(Trial_0001(:,7:11),span,detect,filter);
Trial_0002_s = ligFilter(Trial_0002(:,7:11),span,detect,filter);
Trial_0003_s = ligFilter(Trial_0003(:,7:11),span,detect,filter);
Trial_0004_s = ligFilter(Trial_0004(:,7:11),span,detect,filter);
Trial_0005_s = ligFilter(Trial_0005(:,7:11),span,detect,filter);
Trial_0006_s = ligFilter(Trial_0006(:,7:11),span,detect,filter);
Trial_0007_s = ligFilter(Trial_0007(:,7:11),span,detect,filter);

% combine trials

all_ligaments = vertcat(Trial_0001_s,Trial_0002_s,Trial_0003_s,Trial_0004_s,Trial_0005_s,Trial_0006_s,Trial_0007_s);

LAcH1 = all_ligaments(:,1);
LAcH2 = all_ligaments(:,2);
LCoHd = all_ligaments(:,3);
LScHd = all_ligaments(:,4);
LScHv = all_ligaments(:,5);

% get maximal values

LAcH1_max = max(LAcH1);
LAcH2_max = max(LAcH2);
LCoHd_max = max(LCoHd);
LScHd_max = max(LScHd);
LScHv_max = max(LScHv);


%% Plot all ligaments

TrialLength = length(Trial_0001);

figure;
plot(all_ligaments);
xlim([0 7*TrialLength]);
xl = xline([0 TrialLength TrialLength*2 TrialLength*3 TrialLength*4 TrialLength*5 TrialLength*6],'-',{'Trial 1','Trial 2','Trial 3', 'Trial 4', 'Trial 5', 'Trial 6', 'Trial 7'},'LabelVerticalAlignment','bottom');

xlabel('Frames');
ylabel('Ligament length [mm]');

legend('LAcH1','LAcH2','LCoHd','LScHd','LScHv', 'Location', 'southoutside', 'Orientation', 'horizontal');

%% get cc pointcloud data

ccData = cc(RawData(:,4:6));

% define point cloud and colors

figure
hold on

Cdata = LScHv; % set accoding to ligament to be plotted

cmap = parula;

% make it into a index image.

cmin = min(Cdata(:));
cmax = max(Cdata(:));
m = length(cmap);
index = fix((Cdata-cmin)/(cmax-cmin)*m)+1; %A

% Then to RGB

RGBdata = ind2rgb(index,cmap);
RGB = horzcat((RGBdata(:,:,1)),(RGBdata(:,:,2)),(RGBdata(:,:,3)));

ptCloud = pointCloud(ccData, 'Color', RGB);

pcshow(ptCloud)
set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

hold off

grid on;
axis on;

xlabel('FE_{cc}');
ylabel('ABAD_{cc}');
zlabel('LAR_{cc}');

xlim([-180 180]);
ylim([-90 90]);
zlim([-180 180]);

clim([cmin cmax]);
c = colorbar;
colormap(parula);
view (135,25);

c.Label.String ='LScHv length [mm]'; % adjust label for plot


%% ==== Custom Helper Functions ==== %%

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

%% Ligament Data Filter function

function ligData = ligFilter(inputData,span,detect,filter)

Trial_no = filloutliers(inputData,'makima',detect,span*3,1); % filter outliers using makima
Trial_no = filloutliers(Trial_no,'makima',detect,span*3,1); % second round of filtering 
ligData = smoothdata(Trial_no,1,filter,span); % smooth data using filter

end
