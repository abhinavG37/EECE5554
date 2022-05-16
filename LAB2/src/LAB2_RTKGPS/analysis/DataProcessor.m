clear all
close all
%% Static data processing block

[static_file,path] = uigetfile;

static_bag = rosbag(fullfile(path,static_file));

% bSel = select(static_bag,'Topic','/RTK_GPS/LocationData');
bSel = select(static_bag,'Topic','/GPSLAB2');

static_msgStructs = readMessages(bSel,'DataFormat','struct');

LATStatic   = zeros(0,'double');
LONGStatic  = zeros(0,'double');
UTMN_static = zeros(0,'double');
UTME_static = zeros(0,'double');
ALT_static = zeros(0,'double');
QUAL_static = zeros(0,'double');
for i = 1:length(static_msgStructs)
    LATStatic   = [LATStatic;static_msgStructs{i}.Latitude]; %#ok<*AGROW> 
    LONGStatic  = [LONGStatic;static_msgStructs{i}.Longitude];
    UTMN_static = [UTMN_static;static_msgStructs{i}.UTMNorthing];
    UTME_static = [UTME_static;static_msgStructs{i}.UTMEasting];
    ALT_static  = [ALT_static;static_msgStructs{i}.Altitude];
    QUAL_static = [QUAL_static;static_msgStructs{i}.Quality];
end


%% Non Static data processing block

[nonstatic_file,path] = uigetfile;

nonStaticbag = rosbag(fullfile(path,nonstatic_file));

% bSel = select(static_bag,'Topic','/RTK_GPS/LocationData');
bSel = select(nonStaticbag,'Topic','/GPSLAB2');

motion_msgStructs = readMessages(bSel,'DataFormat','struct');
LATMotion   = zeros(0,'double');
LONGMotion  = zeros(0,'double');
UTMN_motion = zeros(0,'double');
UTME_motion = zeros(0,'double');
ALT_motion = zeros(0,'double');
QUAL_motion = zeros(0,'double');
for i = 1:length(motion_msgStructs)
    LATMotion   = [LATMotion;motion_msgStructs{i}.Latitude]; %#ok<*AGROW> 
    LONGMotion  = [LONGMotion;motion_msgStructs{i}.Longitude];
    UTMN_motion = [UTMN_motion;motion_msgStructs{i}.UTMNorthing];
    UTME_motion = [UTME_motion;motion_msgStructs{i}.UTMEasting];
    ALT_motion  = [ALT_motion;motion_msgStructs{i}.Altitude];
    QUAL_motion = [QUAL_motion;static_msgStructs{i}.Quality];
end
%% Clear clutter in workspace
clear bSel fontSize i nonstatic_file nonStaticbag path static_bag static_file

%% CHANGE THESE NUMBERS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
UTME_static2 = mod(UTME_static,328000);
UTMN_static2 = mod(UTMN_static,468900);
% QUAL_static = QUAL_static;

UTME_motion2  = mod(UTME_motion,328000);
UTMN_motion2  = mod(UTMN_motion,468900);
% QUAL_motion   = QUAL_motion;


%% FOR PLOTTING, REMOVE FIRST 3 numbers since they do not change in UTMS to improve visibility'
%Using the modulus command

% UTME_static2;         Already Declared above
% UTMN_static2;         Already Declared above
% UTME_motion2;         Already Declared above
% UTMN_motion2;         Already Declared above
% QUAL_static;          Already Declared above
% QUAL_motion;          Already Declared above
% ALT_static;           Already Declared above
% ALT_motion;           Already Declared above
QualityIndex =  {'GNSSFix', 'QualVal2', 'QualVal3', 'RTKFix', 'RTKFloat' };
Quality_plot_colors.(QualityIndex{1})=[0 0.4470 0.1];
Quality_plot_colors.(QualityIndex{2})=[0.4, 0.4,0.4];
Quality_plot_colors.(QualityIndex{3})=[0.2, 0.2,0.2];
Quality_plot_colors.(QualityIndex{4})=[0.4, 0.4,0.8];
Quality_plot_colors.(QualityIndex{5})=[0.8, 0.1,0.8];

%% Save workspace
save(fullfile("./MAT", "Workspace.mat"))

%% 3D Scatter plots for UTM coordinates (truncated to meter values only)
%% Static 3D GPS Plot
figure('Name','Static 3D GPS Data ')
H_staticUtm = gcf();
for i=1:length(UTME_static2)
    scatter3(UTME_static2(i), UTMN_static2(i),ALT_static(i), 'filled','Marker','o',...
        'MarkerFaceColor',Quality_plot_colors.(QualityIndex{QUAL_static(i)}), ...
        'MarkerFaceAlpha','0.7');
    hold on
end
p1 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{1}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p2 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{2}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p3 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{3}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p4 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{4}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p5 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{5}),'filled','Marker','o','MarkerFaceAlpha','0.7');
legend([p1, p2, p3, p4, p5],QualityIndex);
grid on
xlabel("UTM Easting [m]")
ylabel("UTM Northing [m]")
zlabel("Altitude [m]")

title("Static 3D GPS Data")
set(gca, 'FontSize', 15, 'TitleFontWeight','bold', 'TitleFontSizeMultiplier',1.5, ...
                                               'FontName', 'Times')
savefig(H_staticUtm, './Fig/StaticUTMPlot_3D_Truncated.fig')

close(H_staticUtm)
%% Non Static 3D GPS Plot
figure('Name','Non-Static 3D GPS Data')
H_nonstaticUtm = gcf();

for i=1:length(UTME_motion2)
    scatter3(UTME_motion2(i), UTMN_motion2(i), ALT_motion(i), 'filled','Marker','o',...
        'MarkerFaceColor',Quality_plot_colors.(QualityIndex{QUAL_motion(i)}), ...
        'MarkerFaceAlpha','0.7');
    hold on
end
p1 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{1}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p2 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{2}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p3 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{3}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p4 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{4}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p5 = scatter3(nan, nan,nan,[], Quality_plot_colors.(QualityIndex{5}),'filled','Marker','o','MarkerFaceAlpha','0.7');
legend([p1, p2, p3, p4, p5],QualityIndex);
grid on
xlabel("UTM Easting [m]")
ylabel("UTM Northing [m]")
zlabel("Altitude [m]")
title("Non-Static 3D GPS Data")
set(gca, 'FontSize', 15, 'TitleFontWeight','bold', 'TitleFontSizeMultiplier',1.5, ...
                                                                    'FontName', 'Times')
savefig(H_nonstaticUtm, './Fig/NonStaticUTMPlot_3D_Truncated.fig')
close(H_nonstaticUtm)


%% 2D Scatter plots for UTM coordinates (truncated to meter values only)
%% Static Case 2D
figure('Name','Static 2D GPS Data')
H_staticUtm = gcf();

for i=1:length(UTME_static2)
    scatter(UTME_static2(i), UTMN_static2(i),'filled','Marker','o', ...
        'MarkerFaceColor',Quality_plot_colors.(QualityIndex{QUAL_static(i)}), ...
        'MarkerFaceAlpha','0.7');
    hold on
end

p1 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{1}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p2 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{2}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p3 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{3}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p4 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{4}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p5 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{5}),'filled','Marker','o','MarkerFaceAlpha','0.7');
legend([p1, p2, p3, p4, p5],QualityIndex);

grid on
xlabel("UTM Easting [m]")
ylabel("UTM Northing [m]")
zlabel("Altitude [m]")
title("Static 2D GPS Data")
set(gca, 'FontSize', 15, 'TitleFontWeight','bold', 'TitleFontSizeMultiplier',1.5, ...
                                                        'FontName', 'Times')
savefig(H_staticUtm, './Fig/StaticUTMPlot_2D_Truncated.fig')
close(H_staticUtm)

%% Non-Static Case 2D
figure('Name','Non Static 2D GPS Data')
H_nonstaticUtm = gcf();

for i=1:length(UTME_motion2)
    scatter(UTME_motion2(i), UTMN_motion2(i),'filled','Marker','o', ...
        'MarkerFaceColor',Quality_plot_colors.(QualityIndex{QUAL_motion(i)}), ...
        'MarkerFaceAlpha','0.7');
    hold on
end
p1 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{1}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p2 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{2}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p3 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{3}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p4 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{4}),'filled','Marker','o','MarkerFaceAlpha','0.7');
p5 = scatter(nan, nan, [], Quality_plot_colors.(QualityIndex{5}),'filled','Marker','o','MarkerFaceAlpha','0.7');
legend([p1, p2, p3, p4, p5],QualityIndex);
 
grid on
xlabel("UTM Easting [m]")
ylabel("UTM Northing [m]")
title("Non-Static 2D GPS Data")
set(gca, 'FontSize', 15, 'TitleFontWeight','bold', 'TitleFontSizeMultiplier',1.5, ...
                                                            'FontName', 'Times')
best_fit_Coeffs = polyfit(UTME_motion2,UTMN_motion2, length(UTME_motion2)-1);
UTMEFit =linspace(min(UTME_motion2), max(UTME_motion2), size(UTME_motion2,1));
UTMNFit =linspace(min(UTMN_motion2), max(UTMN_motion2), size(UTMN_motion2,1));

y1 = polyval(best_fit_Coeffs,UTMEFit);
hold on
plot(UTMEFit,y1,'r-', 'LineWidth',2)

savefig(H_nonstaticUtm, './Fig/NonStaticUTMPlot_2D_Truncated.fig')
close(H_nonstaticUtm)

