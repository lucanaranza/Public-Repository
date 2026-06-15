clc
clear
close all

%% Load data
data = readmatrix('rocket_data.csv'); % columns: Time, Altitude, Roll (radians)
time = data(:,1);
alt  = data(:,2);
roll = data(:,3);

%% Downsample to ~60 fps
dt = mean(diff(time));
fps = 60;
step = max(1, round((1/fps)/dt));
time_ds = time(1:step:end);
alt_ds  = alt(1:step:end);
roll_ds = roll(1:step:end);

%% Load STL using triangulation-aware approach
stl_data = stlread('rocket.stl'); % triangulation object
verts = stl_data.Points;          % numeric Nx3
faces = stl_data.ConnectivityList; % numeric Mx3

% Convert mm -> m if needed
verts = verts / 1000;

% Center around origin
verts_centered = verts - mean(verts,1);

%% Video writer
v = VideoWriter('rocket_roll_stl.mp4','MPEG-4');
v.FrameRate = fps;
open(v);

%% Figure (offscreen)
fig = figure('Color','w','Visible','off','Position',[100 100 1920 1080]);
axis equal
grid on
hold on
set(gca,'Color','w','XColor','k','YColor','k','ZColor','k'); % white bg, black axes
traj_line = plot3(0,0,0,'b','LineWidth',2); % trajectory line

%% Initial patch
rocket_patch = patch('Vertices', verts_centered, 'Faces', faces, ...
                     'FaceColor','k','EdgeColor','none');

%% Main loop
for i = 1:length(time_ds)
    theta = roll_ds(i); % roll in radians
    R = [cos(theta) -sin(theta) 0;
         sin(theta)  cos(theta) 0;
         0           0          1];
    
    % Rotate
    verts_rot = (R * verts_centered.').'; 
    % Translate along Z (scaled for visibility if needed)
    verts_rot(:,3) = verts_rot(:,3) + alt_ds(i); 
    
    set(rocket_patch,'Vertices',verts_rot);
    
    % Update trajectory (X=Y=0, Z=altitude)
    set(traj_line,'XData', zeros(1,i), ...
                   'YData', zeros(1,i), ...
                   'ZData', alt_ds(1:i));
    
    % Dynamic zoom around rocket
    zoom_factor = 1.5; % slightly more zoomed out
    xlim([-zoom_factor zoom_factor]); 
    ylim([-zoom_factor zoom_factor]);
    zlim([max(0,alt_ds(i)-0.5) alt_ds(i)+1.2]);
    
    view(30,20); % fixed camera
    
    % Capture frame
    frame = getframe(fig);
    writeVideo(v, frame);
end

close(v);
close(fig);
disp('MP4 saved: rocket from STL, zoomed out, trajectory, white bg!');
