clc
clear
close all

% --- Find all .eng files in the current folder ---
files = dir('*.eng');

if isempty(files)
    error('No .eng files found in the current folder.');
end

% --- List them ---
fprintf('Available motor files:\n');
for i = 1:length(files)
    fprintf('%d) %s\n', i, files(i).name);
end

% --- User selects one ---
choice = input('Select a motor file (number): ');

if choice < 1 || choice > length(files)
    error('Invalid selection.');
end

filename = files(choice).name;
fprintf('Loading: %s\n', filename);

% --- Read file ---
raw = fileread(filename);
lines = splitlines(raw);

% --- Parse: find the data lines (skip header + comments) ---
thrustData = [];
for i = 1:length(lines)
    line = strtrim(lines{i});
    if isempty(line) || startsWith(line,';')
        continue; % skip comments and blank lines
    end

    nums = sscanf(line, '%f %f');
    if numel(nums) == 2
        thrustData = [thrustData; nums'];
    end
end

% thrustData(:,1) = time (s)
% thrustData(:,2) = thrust (N)

% --- Done ---
t_thrust = thrustData(:,1);
F_thrust = thrustData(:,2);

fprintf('Loaded %d data points.\n', length(t_thrust));

% --- Extract propellant mass from .eng header ---
header = split(lines{1});        % first line of .eng
m_prop = str2double(header{5});  % 4th entry = propellant mass [kg]

% --- Compute total impulse ---
I_total = trapz(t_thrust, F_thrust);   % [Ns]

% --- Compute specific impulse ---
g0 = 9.80665;
Isp = I_total / (m_prop * g0);

% --- Mass flow rate at each step ---
g0 = 9.80665;
mdot = F_thrust ./ (Isp * g0);    % [kg/s] same length as thrust

% --- Propellant mass vs time ---
m_prop_t = m_prop - cumtrapz(t_thrust, mdot);

% Avoid negative mass due to numerical drift
m_prop_t(m_prop_t < 0) = 0;

m0 = 2.194;


fprintf('Propellant mass: %.4f kg\n', m_prop);
fprintf('Total Impulse: %.2f Ns\n', I_total);
fprintf('Specific Impulse: %.2f s\n', Isp);

disp("Sent Data to Simulink ! ...");


%% --- Load Simulink data ---
simOut = sim('Roll_Control_2DOF.slx');  % Run simulation

roll_vector = simOut.logsout.getElement('roll_angle').Values.Data;  % radians
time_vector = simOut.logsout.getElement('roll_angle').Values.Time;
z_vector = simOut.logsout.getElement('altitude').Values.Data;

% Combine data into a table
T = table(time_vector(:), z_vector(:), roll_vector(:), ...
    'VariableNames', {'Time', 'Altitude', 'Roll'});

% Save as CSV
writetable(T, 'rocket_data.csv');
disp('Data exported to rocket_data.csv');
