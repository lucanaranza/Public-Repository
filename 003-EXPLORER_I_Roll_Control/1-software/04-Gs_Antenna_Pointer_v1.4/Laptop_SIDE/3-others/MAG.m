clc
clear
close all

x_data = [-193.05, -187.54, -182.31, -176.56, -168.64, -161.85, -153.74, -144.93, -133.89, -122.52, -112.59, -102.84, -94.01, -87.2, -80.26, -74.05, -68.2, -62.36, -57.73, -53.25, -50.06, -46.45, -43.32, -40.21, -37.21, -34.36, -31.81, -29.2, -26.45, -24.56, -22.41, -19.68, -17.59, -15.29, -13.45, -11.13, -9.79, -7.95, -6.08, -4.14, -2.46, -1.22, -0.03, 0.69, 0.9, 1.27, 1.19, -1.19, -1.27, -0.9, -0.69, 0.03, 1.22, 2.46, 4.14, 6.08, 7.95, 9.79, 11.13, 13.45, 15.29, 17.59, 19.68, 22.41, 24.56, 26.45, 29.2, 31.81, 34.36, 37.21, 40.21, 43.32, 46.45, 50.06, 53.25, 57.73, 62.36, 68.2, 74.05, 80.26, 87.2, 94.01, 102.84, 112.59, 122.52, 133.89, 144.93, 153.74, 161.85, 168.64, 176.56, 182.31, 187.54, 193.05];
y_data = [-178.78, -177, -174.81, -170.8, -167.34, -164.14, -160.92, -161.57, -158.83, -155.77, -151.42, -146.92, -141.53, -137.76, -129.49, -127.15, -121.64, -116.81, -111.46, -104.18, -98.9, -96.26, -92.48, -88.02, -83.82, -82.46, -80.5, -76.5, -73.33, -69.23, -63.47, -58.63, -54.25, -50.87, -47.44, -44.27, -41.66, -38.7, -35.08, -29.21, -23.54, -18.89, -13.96, -9.73, -5.83, -1.73, -0.11, 0.11, 1.73, 5.83, 9.73, 13.96, 18.89, 23.54, 29.21, 35.08, 38.7, 41.66, 44.27, 47.44, 50.87, 54.25, 58.63, 63.47, 69.23, 73.33, 76.5, 80.5, 82.46, 83.82, 88.02, 92.48, 96.26, 98.9, 104.18, 111.46, 116.81, 121.64, 127.15, 129.49, 137.76, 141.53, 146.92, 151.42, 155.77, 158.83, 161.57, 160.92, 164.14, 167.34, 170.8, 174.81, 177, 178.78];

% Segment 1: 0 to +50
mask1 = x_data >= 0 & x_data <= 50;
[p1, ~, mu1] = polyfit(x_data(mask1), y_data(mask1), 5);

% Segment 2: 50 to +200
mask2 = x_data > 50;
[p2, ~, mu2] = polyfit(x_data(mask2), y_data(mask2), 5);

% Expand to single coefficients using symbolic math
syms x t
expr1 = subs(poly2sym(p1, t), t, (x - mu1(1)) / mu1(2));
expr2 = subs(poly2sym(p2, t), t, (x - mu2(1)) / mu2(2));
p1_exp = sym2poly(expand(expr1));
p2_exp = sym2poly(expand(expr2));

fprintf('Seg1 expanded: '); fprintf('%.10f ', p1_exp); fprintf('\n');
fprintf('Seg2 expanded: '); fprintf('%.10f ', p2_exp); fprintf('\n');

% Evaluate fits for plotting
xq1 = linspace(0, 50, 500);
yq1 = polyval(p1, (xq1 - mu1(1)) / mu1(2));
xq2 = linspace(50, 200, 500);
yq2 = polyval(p2, (xq2 - mu2(1)) / mu2(2));

% Symmetric segments (negate)
xq3 = -xq1;  yq3 = -yq1;
xq4 = -xq2;  yq4 = -yq2;

% Plot
figure; hold on;
plot(x_data, y_data, 'k.', 'MarkerSize', 10, 'DisplayName', 'Data');
plot(xq1,  yq1,  'r-',  'LineWidth', 2, 'DisplayName', 'Seg1: 0 to +50');
plot(xq2,  yq2,  'b-',  'LineWidth', 2, 'DisplayName', 'Seg2: +50 to +200');
plot(xq3,  yq3,  'm--', 'LineWidth', 2, 'DisplayName', 'Seg3: 0 to -50');
plot(xq4,  yq4,  'c--', 'LineWidth', 2, 'DisplayName', 'Seg4: -50 to -200');
grid on; axis equal;
legend('Location', 'northwest');
title('Direct Polyfit y = f(x) per Segment');
xlabel('x'); ylabel('y');

% Evaluate using expanded coefficients at all 4 segments
xq1_check = linspace(0, 50, 500);
yq1_check = polyval(p1_exp, xq1_check);

xq2_check = linspace(50, 200, 500);
yq2_check = polyval(p2_exp, xq2_check);

xq3_check = -xq1_check;  yq3_check = -yq1_check;
xq4_check = -xq2_check;  yq4_check = -yq2_check;

figure; hold on;
plot(x_data, y_data, 'k.', 'MarkerSize', 10, 'DisplayName', 'Data');
plot(xq1_check,  yq1_check,  'r-',  'LineWidth', 2, 'DisplayName', 'Seg1: 0 to +50');
plot(xq2_check,  yq2_check,  'b-',  'LineWidth', 2, 'DisplayName', 'Seg2: +50 to +200');
plot(xq3_check,  yq3_check,  'm--', 'LineWidth', 2, 'DisplayName', 'Seg3: 0 to -50');
plot(xq4_check,  yq4_check,  'c--', 'LineWidth', 2, 'DisplayName', 'Seg4: -50 to -200');
grid on; axis equal;
legend('Location', 'northwest');
title('Expanded Coefficients Check - y = f(x)');
xlabel('x'); ylabel('y');

y_calc = zeros(size(x_data));
for i = 1:length(x_data)
    x_abs = abs(x_data(i));
    s = sign(x_data(i));
    if x_abs <= 50
        t = (x_abs - mu1(1)) / mu1(2);
        y_calc(i) = s * polyval(p1, t);
    else
        t = (x_abs - mu2(1)) / mu2(2);
        y_calc(i) = s * polyval(p2, t);
    end
end

figure;
subplot(2,1,1);
plot(x_data, y_data, 'k.', x_data, y_calc, 'r-');
grid on; legend('Data','Fit'); title('Fit vs Data');

subplot(2,1,2);
plot(x_data, y_data - y_calc, 'b-');
grid on; title('Error'); xlabel('x'); ylabel('error (deg)');
fprintf('Max error: %.2f\n', max(abs(y_data - y_calc)));