%% ============================================================
% TELEMETRY ANALYSIS
% =============================================================

clear;
clc;
close all;

%% Load data

data = readtable('2-data_files_SD/data.txt');
% Skip first 7 lines
data = data(45:end, :);

% MUST be defined here
vars = data.Properties.VariableNames;

%% Extract variables

om     = data.om;
fq     = data.fq;
ts     = data.ts;

ax     = data.ax;
ay     = data.ay;
az     = data.az;

gx     = data.gx;
gy     = data.gy;
gz     = data.gz;

p      = data.p;
y      = data.y;
r      = data.r;

al     = data.al;
vz     = data.vz;

S1_pos = data.S1_pos;

tp     = data.tp;

x_pos  = data.x_pos;
y_pos  = data.y_pos;

sat    = data.sat;
v_bat  = data.v_bat;

LW = 1.0;

%% ============================================================
% 1-ALTITUDE
% =============================================================
figure('Position',[100 100 1600 800],'Name','Altitude');

plot(ts,al,'k','LineWidth',LW);

grid on;
title('Altitude');
xlabel('Time [s]');
ylabel('Altitude [m]');

%% ============================================================
% 2-ATTITUDE
% =============================================================
figure('Position',[100 100 1600 800],'Name','Attitude');

plot(ts,p,'r','LineWidth',LW); hold on;
plot(ts,y,'g','LineWidth',LW);
plot(ts,r,'b','LineWidth',LW);

grid on;
title('Pitch / Yaw / Roll');
xlabel('Time [s]');
ylabel('Angle [deg]');
legend('Pitch','Yaw','Roll');
hold off;

%% ============================================================
% 3-VERTICAL VELOCITY
% =============================================================
figure('Position',[100 100 1600 800],'Name','Vertical Velocity');

plot(ts,vz,'k','LineWidth',LW);

grid on;
title('Vertical Velocity');
xlabel('Time [s]');
ylabel('V_z [m/s]');

%% ============================================================
% 4-ACCELERATIONS
% =============================================================
figure('Position',[100 100 1600 800],'Name','Accelerations');

plot(ts,ax,'r','LineWidth',LW); hold on;
plot(ts,ay,'g','LineWidth',LW);
plot(ts,az,'b','LineWidth',LW);

grid on;
title('Accelerations');
xlabel('Time [s]');
ylabel('Acceleration');
legend('a_x','a_y','a_z');
hold off;

%% ============================================================
% 5-ANGULAR RATES
% =============================================================
figure('Position',[100 100 1600 800],'Name','Angular Rates');

plot(ts,gx,'r','LineWidth',LW); hold on;
plot(ts,gy,'g','LineWidth',LW);
plot(ts,gz,'b','LineWidth',LW);

grid on;
title('Angular Rates');
xlabel('Time [s]');
ylabel('Rate [deg/s]');
legend('g_x','g_y','g_z');
hold off;

%% ============================================================
% 6-X/Y POSITION
% =============================================================
figure('Position',[100 100 1600 800],'Name','XY Position against time');

plot(ts,x_pos,'r','LineWidth',LW); hold on;
plot(ts,y_pos,'g','LineWidth',LW);

grid on;
title('Position');
xlabel('Time [s]');
ylabel('Position [m]');
legend('X Position','Y Position');
hold off;

%% ============================================================
% 7-X/Y POSITION
% =============================================================
figure('Position',[100 100 1600 800],'Name','XY Position');

plot(x_pos,y_pos,'r','LineWidth',LW);

grid on;
title('Position');
xlabel("X Position [m]");
ylabel('Y Position [m]');
legend('X Position','Y Position');
hold off;

%% ============================================================
% 8-ROLL + SERVO POSITION
% =============================================================
figure('Position',[100 100 1600 800],'Name','Roll and Servo');

plot(ts,r,'b','LineWidth',LW); hold on;
plot(ts,S1_pos,'k','LineWidth',LW);

grid on;
title('Roll and Servo Position');
xlabel('Time [s]');
ylabel('Value');
legend('Roll','S1 Position');
hold off;

for i = 1:length(vars)

    vname = vars{i};
    y = data.(vname);
    figure('Position',[100 100 1600 800]);
    plot(ts, y, 'LineWidth', 1.5);
    grid on;
    title(vname, 'Interpreter', 'none');
    xlabel("Time (s)");
    ylabel(vname);

end