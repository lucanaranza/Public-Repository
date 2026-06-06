clc;
clear;
close all;

%% SERIAL SETUP
s = serialport("COM6", 500000);
configureTerminator(s, "LF");

flush(s);

%% LABELS (your full packet)
labels = {
"targetAz","targetEl","distance","mode", ...
"packet_id","om","fq","sd","ts", ...
"ax","ay","az", ...
"gx","gy","gz", ...
"p","y","r", ...
"al","vz","S1_pos", ...
"tp","x_pos","y_pos","sat","v_bat"
};

N = length(labels);

%% DATA STORAGE
WINDOW = 200;

data = NaN(WINDOW, N);
i = 0;

%% FIGURES (one per variable)
figs = gobjects(N,1);
plots = gobjects(N,1);

for k = 1:N
    figs(k) = figure('Name', labels{k});
    plots(k) = animatedline;
    grid on;
    xlabel("sample");
    ylabel(labels{k});
end

%% MAIN LOOP
while true

    line = readline(s);
    vals = str2double(split(strtrim(line), ","));

    if length(vals) ~= N
        continue;
    end

    i = i + 1;
    idx = mod(i-1, WINDOW) + 1;
    data(idx,:) = vals;

    for k = 1:N
    y = data(:,k);

    % reorder so time is correct (ring buffer fix)
    if i > WINDOW
        y = circshift(y, -idx);
    end

    clearpoints(plots(k));
    addpoints(plots(k), 1:WINDOW, y);
end

    drawnow limitrate
end