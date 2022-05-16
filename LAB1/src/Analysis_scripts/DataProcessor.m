clear all
close all
%% Script to generate all working data from the recorded bag files and store the plots as .fig files

%% Static data processing block

[static_file,path] = uigetfile;

static_bag = rosbag(fullfile(path,static_file));

bSel = select(static_bag,'Topic','/RTK_GPS/LocationData');

static_msgStructs = readMessages(bSel,'DataFormat','struct');

LATStatic   = zeros(0,'double');
LONGStatic  = zeros(0,'double');
UTMN_static = zeros(0,'double');
UTME_static = zeros(0,'double');
ALT_static = zeros(0,'double');

for i = 1:length(static_msgStructs)
    LATStatic   = [LATStatic;static_msgStructs{i}.Latitude]; %#ok<*AGROW> 
    LONGStatic  = [LONGStatic;static_msgStructs{i}.Longitude];
    UTMN_static = [UTMN_static;static_msgStructs{i}.UtmNorthing];
    UTME_static = [UTME_static;static_msgStructs{i}.UtmEasting];
    ALT_static = [ALT_static;static_msgStructs{i}.Altitude];
end

UTME_static2 = mod(UTME_static,325000);
UTMN_static2 = mod(UTMN_static,468700);



%% Non Static data processing block

[nonstatic_file,path] = uigetfile;

nonStaticbag = rosbag(fullfile(path,nonstatic_file));

bSel = select(nonStaticbag,'Topic','/RTK_GPS/LocationData');
motion_msgStructs = readMessages(bSel,'DataFormat','struct');
LATMotion   = zeros(0,'double');
LONGMotion  = zeros(0,'double');
UTMN_motion = zeros(0,'double');
UTME_motion = zeros(0,'double');
ALT_motion = zeros(0,'double');
for i = 1:length(motion_msgStructs)
    LATMotion   = [LATMotion;motion_msgStructs{i}.Latitude]; %#ok<*AGROW> 
    LONGMotion  = [LONGMotion;motion_msgStructs{i}.Longitude];
    UTMN_motion = [UTMN_motion;motion_msgStructs{i}.UtmNorthing];
    UTME_motion = [UTME_motion;motion_msgStructs{i}.UtmEasting];
    ALT_motion = [ALT_motion;static_msgStructs{i}.Altitude];
end

UTME_motion2  = mod(UTME_motion,325000);
UTMN_motion2  = mod(UTMN_motion,468700);


%% Block to plot latitude and longitudes without zone digit truncation
% Not used

% figure('Name','Static GPS Data Plot')
% plot(LONGStatic, LATStatic, 'Marker','*', 'Color','b', 'LineStyle','-', 'LineWidth',2)
% xlabel("Latitude (DD.DDDD)")
% ylabel("Longitude (DD.DDDD)")
% H_static = gcf();
% savefig(H_static, 'Fig/StaticPlot.fig')
% close(H_static)


% figure('Name','Non Static GPS Data')
% plot(LONGMotion, LATMotion, 'Marker','*', 'Color','b', 'LineStyle','-', 'LineWidth',2)
% xlabel("Latitude (DD.DDDD)")
% ylabel("Longitude (DD.DDDD)")
% H_nonstatic = gcf();
% savefig(H_nonstatic, 'Fig/NonStaticPlot.fig')
% close(H_nonstatic)

%% FOR PLOTTING, REMOVE FIRST 3 numbers since they do not change in UTMS to improve visibility'
%Using the modulus command

% UTME_static2;         Already Declared above
% UTMN_static2;         Already Declared above
% UTME_motion2;         Already Declared above
% UTMN_motion2;         Already Declared above
% ALT_static;           Already Declared above
% ALT_motion;           Already Declared above

%% 3D Scatter plots for UTM coordinates (truncated to meter values only)
figure('Name','Static 3D GPS Data ')
H_staticUtm = gcf();
scatter3(UTME_static2, UTMN_static2,ALT_static, 'filled','Marker','o', 'MarkerFaceColor','[0.2 0.7 0.4]','MarkerFaceAlpha','0.3');
grid on
xlabel("UTM Easting [m]")
ylabel("UTM Northing [m]")
zlabel("Altitude [m]")
title("Static 3D GPS Data")
set(gca, 'FontSize', 15, 'TitleFontWeight','bold', 'TitleFontSizeMultiplier',1.5, ...
                                                    'FontName', 'Times')
savefig(H_staticUtm, 'Fig/StaticUTMPlot_3D_Truncated.fig')
close(H_staticUtm)

figure('Name','Non-Static 3D GPS Data')
H_nonstaticUtm = gcf();
scatter3(UTME_motion2, UTMN_motion2, ALT_motion, 'filled','Marker','o', 'MarkerFaceColor','[0.2 0.7 0.4]','MarkerFaceAlpha','0.3');
grid on
xlabel("UTM Easting [m]")
ylabel("UTM Northing [m]")
zlabel("Altitude [m]")
title("Non-Static 3D GPS Data")
set(gca, 'FontSize', 15, 'TitleFontWeight','bold', 'TitleFontSizeMultiplier',1.5, ...
                                                                    'FontName', 'Times')
savefig(H_nonstaticUtm, 'Fig/NonStaticUTMPlot_3D_Truncated.fig')
close(H_nonstaticUtm)


%% 2D Scatter plots for UTM coordinates (truncated to meter values only)
figure('Name','Static 2D GPS Data')
H_staticUtm = gcf();
scatter(UTME_static2, UTMN_static2,'filled', 'Marker','o', 'Color','b', 'MarkerFaceAlpha','0.3');
grid on
xlabel("UTM Easting [m]")
ylabel("UTM Northing [m]")
zlabel("Altitude [m]")
title("Non-Static 2D GPS Data")
set(gca, 'FontSize', 15, 'TitleFontWeight','bold', 'TitleFontSizeMultiplier',1.5, ...
                                                        'FontName', 'Times')
savefig(H_staticUtm, 'Fig/StaticUTMPlot_2D_Truncated.fig')
close(H_staticUtm)

figure('Name','Non Static 2D GPS Data')
H_nonstaticUtm = gcf();
scatter(UTME_motion2, UTMN_motion2,'filled', 'Marker','o', 'Color','b','MarkerFaceAlpha','0.3');
grid on
xlabel("UTM Easting [m]")
ylabel("UTM Northing [m]")
title("Non-Static 2D GPS Data")
set(gca, 'FontSize', 15, 'TitleFontWeight','bold', 'TitleFontSizeMultiplier',1.5, ...
                                                            'FontName', 'Times')
savefig(H_nonstaticUtm, 'Fig/NonStaticUTMPlot_2D_Truncated.fig')
close(H_nonstaticUtm)

