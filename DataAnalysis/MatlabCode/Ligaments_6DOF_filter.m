% Ligaments_6DOF.m
%   This script reduces 6DOF ROM and ligament simulations from Maya into 
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

%% load data

% load XROMM data

XROMMData = csvread('RLPs_XROMM_rot.csv',1,0); % get translational data

% load 6DOF simulation data

RLP3_6DOF_trans = readtable("RLP3_6DOF_trans.csv"); % get translational data
RLP3_6DOF_rot = readtable("RLP3_6DOF_rot.csv"); % get rotational and viable data
RLP3_6DOF = horzcat(RLP3_6DOF_trans, RLP3_6DOF_rot); % combine translational and viable data
RLP3_6DOF = round(RLP3_6DOF,4); % round data for merging
RLP3_6DOF.sort = transpose(1:height(RLP3_6DOF));

% set prefeix for ligament data import

prefix = "RLP3_lig_part_";

% import ligament csv files

first = 1; % first element in series of files
last = 88; % last element in series of files
numRange = first:1:last;

% go through specified ligament files

for i = 1:length(numRange)
    
    % import data from ligament part csv files

    if numRange(i) < 10
        filename = prefix + "0" + int2str(numRange(i)) + ".csv";
    else
        filename = prefix + int2str(numRange(i)) + ".csv";
    end
    
    % combine data from ligament files

    if i == 1
        T = readtable(filename); % read in data
        T = round(T,4); % round data for merging later
    else
        T1 = readtable(filename); % read in data
        T1 = round(T1,4); % round data for merging later

        % combine datasets

        T = [T;T1];
    end

end  

T = unique(T,'rows');
T = renamevars(T,["myJoint_tx","myJoint_ty","myJoint_tz","myJoint_rx","myJoint_ry","myJoint_rz"], ...
                 ["tX","tY","tZ","rX","rY","rZ"]);

RLP3_6DOF = renamevars(RLP3_6DOF,["myJoint_hum_tx","myJoint_hum_ty","myJoint_hum_tz","myJoint_hum_rx","myJoint_hum_ry","myJoint_hum_rz"], ...
                 ["tX","tY","tZ","rX","rY","rZ"]);

% merge ligament data with 6DOF ROM data

Data = outerjoin(RLP3_6DOF, T, 'Keys',["tX","tY","tZ","rX","rY","rZ"], 'MergeKeys', 1); 
Data = sortrows(Data,8);
Data.sort = [];

DataMat = table2array(Data);

% Temporary to check if filtering works properly

Data_3D = permute(reshape(DataMat,[197173,27,12]),[1,3,2]); 


%% filter and combine ligament data

% set filter parameters 

span = 8;
filter = 'rlowess';
itter = 2;

% filter ligaments and remove outliers

lig1 = ligFilter(DataMat, 8, span, filter, itter);
lig2 = ligFilter(DataMat, 9, span, filter, itter);
lig3 = ligFilter(DataMat, 10, span, filter, itter);
lig4 = ligFilter(DataMat, 11, span, filter, itter);
lig5 = ligFilter(DataMat, 12, span, filter, itter);

lig_3D = horzcat(lig1,lig2,lig3,lig4,lig5);


%% get shortest ligament lengths per pose

NaNRows = permute(any(isnan(lig_3D),2),[1,3,2]);
lig_3D_nan = permute(lig_3D,[1,3,2]);
for i = 1:size(lig_3D,2)
    lig_2D_nan = lig_3D(:,i,:);
    lig_2D_nan(NaNRows) = NaN;
    lig_3D_nan(:,:,i) = lig_2D_nan;
end
lig_3D_nan = permute(lig_3D_nan,[1,3,2]);

lig_3D_nan(:,6,:) = sum(lig_3D_nan(:,1:5,:),2); % get cumulative length of ligaments
[lig_min,lig_ind] = min(lig_3D_nan(:,6,:),[],3); % get index of joint position with smallest cumulative ligament length

lig_2D = zeros(length(lig_3D_nan),5);

% get ligament lengths from pose with smallest cumulative ligament length

for i = 1:length(lig_3D_nan)
    if lig_min(i) > 100 % if cumulative ligament length exceeds this value the pose becomes inviable
        lig_2D(i,:) = NaN(1,5);
    else
        lig_2D(i,:) = lig_3D_nan(i,1:5,lig_ind(i));
    end
end

% create rotaion grid

rotX = unique(DataMat(:,4), 'rows')'; % get set of X rotational values
rotY = unique(DataMat(:,5), 'rows')'; % get set of Y rotational values
rotZ = unique(DataMat(:,6), 'rows')'; % get set of Z rotational values

[xn, yn, zn] = ndgrid(rotX,rotY,rotZ); 
rotData = [xn(:), yn(:), zn(:)];

% combine data into table

ligData = horzcat(rotData,lig_2D);

ligTable = array2table(ligData, ...
    'VariableNames', ...
    {Data.Properties.VariableNames{4}, ...
    Data.Properties.VariableNames{5}, ...
    Data.Properties.VariableNames{6}, ...
    Data.Properties.VariableNames{8}, ...
    Data.Properties.VariableNames{9}, ...
    Data.Properties.VariableNames{10}, ...
    Data.Properties.VariableNames{11}, ...
    Data.Properties.VariableNames{12}});

% cosine correct data

ccData = cc(ligData);

% seperate ligament data
LAcH1 = ligData(:,4);
LAcH2 = ligData(:,5);
LCoHd = ligData(:,6);
LScHd = ligData(:,7);
LScHv = ligData(:,8);

% ==== triangulated minimum stretch (TMS) ==== %

% caluclate sum of absolute ligament lengths
[minlig,minLigRow] = min(lig_min); % get minumum cumulative absolute ligament length

% get individual ligament lengths from minimum cimulative length

stretch = 24.5; % Beier, 2012
lig_stretch = 1/100*(100+stretch);

LAcH1_TMS = ligData(minLigRow,4) * lig_stretch;  % double min length to account for ligament slack
LAcH2_TMS = ligData(minLigRow,5) * lig_stretch;  % double min length to account for ligament slack
LCoHd_TMS = ligData(minLigRow,6) * lig_stretch;  % double min length to account for ligament slack
LScHd_TMS = ligData(minLigRow,7) * lig_stretch;  % double min length to account for ligament slack
LScHv_TMS = ligData(minLigRow,8) * lig_stretch;  % double min length to account for ligament slack

TMS_rot = ligData(minLigRow,1:3);


% get treshold data for ligaments

%ligThresholds = ligData(minLigRow,4:8) * lig_stretch; % TMS values
ligThresholds = [18.85,18.5,9.49,18.85,9.75]; % scaled dissection values
%ligThresholds = [14.92492888,20.20930299,10.87463016,15.70545519,9.345220484]; # XROMM %values


%% constrain ROM based on ligament thresholds

% remove datapoint above thresholds

consData = ligCons(ligTable,ligThresholds);

Deg = 5; %set according to the degree interval in Maya simulation 
minDistance = Deg*3^(1/2);

ptCloudDN = pcdenoise(pointCloud([consData(:,[1 2 3])])); % denoise point cloud
[labels,numClusters] = pcsegdist(ptCloudDN,minDistance); % define clusters
ptCloudDNLocation = ptCloudDN.Location;

% show clusters

figure();

pcshow(ptCloudDNLocation,labels, 'VerticalAxisDir', 'Up', 'VerticalAxis', 'Y', 'MarkerSize', 50); colormap(hsv(numClusters)); %display the different clusters xlabel('Flexion/Extension');
set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
hold off

grid on;
axis on;

xlabel('FE');
ylabel('ABAD');
zlabel('LAR');

xlim([-180 180]);
ylim([-90 90]);
zlim([-180 180]);
view (135,25);

% remove 'islands' from data

ClusterBins = (1:numClusters); %number of clusters
ClusterElementCount = histcounts(labels,numClusters); %elements within each cluster
ElementsPerBin = transpose([ClusterBins; ClusterElementCount]);
ClustersFilteredNum = ElementsPerBin(ElementsPerBin(:,2) >= max(ClusterElementCount),:); % only retains points within largerst cluster
Cluster = select(ptCloudDN,find(labels == ClustersFilteredNum(:,1))); % select largest cluster

% cosine correct data for plotting

ccCluster = cc(Cluster.Location);
ptCloud = pointCloud(ccCluster);

% show final point cloud

figure();

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

colormap(parula);
view (135,25);

% export data

expStr = split(prefix,'_');
exportfilename = append(convertStringsToChars(expStr(1)),'_Dissection_constrained_ROM.csv');
exortData = array2table(Cluster.Location,'VariableNames', ["rX", "rY", "rZ"]);
writetable(exortData,exportfilename);


%% Compare soft tissue constrained simulations with XROMM data

inputData = 'Dissection';

ccXROMM = cc(XROMMData);
ccAXROMM = alphaShape(ccXROMM(:,[1 2 3])); %create a 3-D alpha shape using second set of (FE°, ABAD°, LAR°) points 
ccAXROMMCA = criticalAlpha(ccAXROMM,'all-points');
ccAXROMM.Alpha = ccAXROMMCA;
ccAXROMMVolume = volume(ccAXROMM);

ccROM = cc(Cluster.Location);
ccAROM = alphaShape(ccROM(:,[1 2 3])); %create a 3-D alpha shape using second set of (FE°, ABAD°, LAR°) points 
ccAROMCA = criticalAlpha(ccAROM,'all-points');
ccAROM.Alpha = ccAROMCA;
ccAROMVolume = volume(ccAROM);

x1 = ccXROMM(:,1);
y1 = ccXROMM(:,2);
z1 = ccXROMM(:,3);
x2 = ccROM(:,1);
y2 = ccROM(:,2);
z2 = ccROM(:,3);

% find points in both XROMM and simulation shapes

id1=inShape(ccAROM,x1,y1,z1);
id1Points = [x1(id1), y1(id1), z1(id1)];
id1Shape = alphaShape(id1Points);
id1CA = criticalAlpha(id1Shape,'all-points');
id1Shape.Alpha = ccAXROMMCA;
id1Volume = volume(id1Shape);

% find XROMM data points outside simulation shape

missedPoints = [x1(~id1), y1(~id1), z1(~id1)];
missedShape = alphaShape(missedPoints);
missedCA = criticalAlpha(missedShape,'all-points');
missedShape.Alpha = 10;
missedVolume = volume(missedShape);

missedPercentage = 100 / ccAXROMMVolume * missedVolume;

%report volumes

fprintf(['XROMM volume = ' num2str(ccAXROMMVolume,'%6.3f'),'\n']);
fprintf(['Total ROM volume = ' num2str(ccAROMVolume,'%6.3f'),'\n']);
fprintf(['Missed Volume = ' num2str(missedVolume,'%6.3f'),'\n']);
fprintf(['Missed Volume Percentage = ' num2str(missedPercentage,'%6.3f'),'\n']);

% Plot CC data

fig = figure;

hold on

set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);

Shape1 = plot(missedShape); %plot the first alpha shape on the same axes 
set(Shape1, 'facecolor','#B0191D','edgecolor','#B0191D',  'facealpha', '0.95', 'edgealpha', '0.95'); %set colour and opacity of alpha shape
Shape2 = plot(ccAROM); %plot the second alpha shape on the same axes 
set(Shape2, 'facecolor','#40752D','edgecolor','#40752D',  'facealpha', '0.05', 'edgealpha', '0.05'); %set colour and opacity of alpha shape
Shape3 = plot(ccAXROMM); %plot the second alpha shape on the same axes 
set(Shape3, 'facecolor','#576FB6','edgecolor','#576FB6',  'facealpha', '0.5', 'edgealpha', '0.5'); %set colour and opacity of alpha shape


legend('Missed data',inputData,'XROMM data','Location','southeastoutside');

hold off

xlabel('FE_{cc}');
ylabel('ABAD_{cc}');
zlabel('LAR_{cc}');

xlim([-180 180]);
ylim([-90 90]);
zlim([-180 180]);
clim([-180 180]);

xticks([-180,-120,-60,0,60,120,180])
yticks([-90,-45,0,45,90])
zticks([-180,-120,-60,0,60,120,180])

colormap(flipud(parula));
view (145,25);

grid on;
axis on;

% export figure

exportFileName = append(inputData,'.pdf');
exportgraphics(fig,exportFileName,'ContentType','vector');


%% ==== Custom Helper Functions ==== %%

%% constrain rotational data based on ligament lengths

function consData = ligCons(inputTable,thresholds) % Rotation for X, Y and Z axes and calculated ligament lengths

inputData = table2array(inputTable);

numCol = size(inputData,2);

for i = 4:numCol % get number of ligaments, i.e., data minus rotation columns
    
    if (i == 4) % get data at first itteration 
        consData = inputData;
        consData(any(isnan(inputData),2),:) = []; % remove any NaNs from data
    end
    
    ligData = consData(:,i); % get ligament from data
    
    cmap = parula; % set colormap
    
    % convert data into a index image

    cmin = min(ligData(:));
    cmax = thresholds(i-3);
    m = length(cmap);
    index = fix((ligData-cmin)/(cmax-cmin)*m)+1;

    % Then to RGB

    RGBdata = ind2rgb(index,cmap);
    RGB = horzcat((RGBdata(:,:,1)),(RGBdata(:,:,2)),(RGBdata(:,:,3)));
    
    PlotData = horzcat(consData, RGB);
    consData(consData(:,i) > thresholds(i-3), :) = [];
 
    % set color to red if above threshold

    for k = 1:size(PlotData,1)
        if PlotData(k,i) > thresholds(i-3)
            PlotData(k,numCol+1) = 1.0;
            PlotData(k,numCol+2) = 0.0;
            PlotData(k,numCol+3) = 0.0;
        end
    end

    % create figure

    fig = figure();
    hold on
    ptCloud = pointCloud(cc([PlotData(:,[1 2 3])]), 'Color', [PlotData(:,[9 10 11])]);
    
    pcshow(ptCloud)
    set(gcf,'color','w');set(gca,'color','w');set(gca, 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'ZColor', [0.15 0.15 0.15]);
    
    hold off
    
    grid on;
    axis on;
    
    xlabel('FE_c_c');
    ylabel('ABAD_c_c');
    zlabel('LAR_c_c');
    
    xlim([-180 180]);
    ylim([-90 90]);
    zlim([-180 180]);
    
    clim([cmin cmax]);
    colorbar;
    colormap(parula);
    view (135,25);
    
    c = colorbar;

    labelSplit = split(inputTable.Properties.VariableNames(4:end).','_');
    labelStr = append(convertStringsToChars(strjoin(labelSplit(i-3,2:end))),' length [mm]');

    c.Label.String =labelStr;
    
    % export data

    exportfilename = append(convertStringsToChars(strjoin(labelSplit(i-3,2:end),'_')),'_Ligament_ROM_DIS.pdf');
    %exportgraphics(fig,exportfilename,'ContentType','vector');
end

end

    
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


%% Ligament Data Filter and 6DOF to 3DOF function

function ligData = ligFilter(inputData,ligInd,span,filter,itter) 

posInd = unique(inputData(:,1:3),'rows'); % get unique positions from 6DOF ROM simulation
rotXInd = unique(inputData(:,4), 'rows'); % get set of X rotational values
rotyInd = unique(inputData(:,5), 'rows'); % get set of Y rotational values
rotZInd = unique(inputData(:,6), 'rows'); % get set of Z rotational values

% define ligament variable

lig_4D = []; 

% go through each translational position and filter outliers of ligaments

for j = 1:length(posInd(:,1))
    
    % get rotational dataset for each translational position

    subset = inputData(inputData(:,1) == posInd(j,1) & inputData(:,2) == posInd(j,2) & inputData(:,3) == posInd(j,3),:);
    
    % get ligament 1 and create 3D array based on rotation

    lig = subset(:,ligInd);
    lig_3D = reshape(lig.',length(rotXInd),length(rotyInd),length(rotZInd));
    lig_nan = isnan(lig_3D); % initially missing values

    for k = 1:itter
           
        if k == 1
            lig_min = lig_3D;
        end
    
        % filter outliers accross each DOF

        lig_outliers = isoutlier(lig_min); % find outliers
        lig_neg = lig_min < 0; % find outliers if value is below 0
        lig_threshold = lig_min > 100; % find outliers if above threshold

        lig_arr = cat(4,lig_outliers,lig_neg,lig_threshold); % combine different outlier detection methods

        lig_out = max(lig_arr,[],4); % get outlier logical array

        lig_min(lig_out) = NaN; % set identified outliers to NaN

        % fill missing values (outliers) along each axis

        lig_x_no = fillmissing(lig_min,'makima',1);
        lig_x_no(lig_nan) = NaN; % set initially missing values back to NaN
        lig_y_no = fillmissing(lig_min,'makima',2);
        lig_y_no(lig_nan) = NaN; % set initially missing values back to NaN
        lig_z_no = fillmissing(lig_min,'makima',3);
        lig_z_no(lig_nan) = NaN; % set initially missing values back to NaN

        lig_med = abs(median(cat(4, lig_x_no, lig_y_no, lig_z_no),4));

        % filter outliers accross each DOF round 2 and smooth data

        lig_x_s = smoothdata(lig_med,1,filter,span); % smooth along X-axis
        lig_x_s(lig_nan) = NaN;

        lig_y_s = smoothdata(lig_med,2,filter,span); % smooth along Y-axis
        lig_y_s(lig_nan) = NaN;

        lig_z_s = smoothdata(lig_med,3,filter,span); % smooth along Z-axis
        lig_z_s(lig_nan) = NaN;

        % combine filtered datasets by taking median value accross them  

        lig_min = median(abs(cat(4, lig_x_s, lig_y_s, lig_z_s)),4);

        % if value bigger than simulations set back to simulation results

        lig_diff = lig_min > lig_3D & lig_3D ~= -1; 
        lig_min(lig_diff) = lig_3D(lig_diff); 

    end

    lig_4D = cat(4,lig_4D,lig_min); % concatenate each translational position

end

% combine 6DOF positions into rotational column vetors for each joint position 

ligData = reshape(lig_4D,length(lig),1,length(posInd(:,1)));

end

