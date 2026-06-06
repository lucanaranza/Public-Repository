clc; clear; close all;

gyro_pos = [30, 60, 90, 120, 150, 180];
mag_pos  = [22.5, 43.3, 63, 82.8, 102.5, 125];

gyro_neg = [-30, -60, -90, -120, -150, -180];
mag_neg  = [-27, -57, -111, -176, -219, -244];

% ── Positive: linear fit ──────────────────────────────────────────────
p_pos = polyfit(mag_pos, gyro_pos, 1);
fprintf('Positive linear: gyro = %.6f * mag + %.6f\n', p_pos(1), p_pos(2));

xq_pos = linspace(0, 130, 500);
yq_pos = polyval(p_pos, xq_pos);

% ── Negative: polynomial fit ──────────────────────────────────────────
% flip to positive, fit, then negate
mag_neg_abs  = abs(mag_neg);
gyro_neg_abs = abs(gyro_neg);

[p_neg, ~, mu_neg] = polyfit(mag_neg_abs, gyro_neg_abs, 3);
fprintf('Negative poly degree 3 coeffs: '); fprintf('%.6f ', p_neg); fprintf('\n');
fprintf('Negative mu: %.6f %.6f\n', mu_neg(1), mu_neg(2));

xq_neg = linspace(0, 250, 500);
yq_neg = polyval(p_neg, (xq_neg - mu_neg(1)) / mu_neg(2));

% ── Plot ──────────────────────────────────────────────────────────────
figure; hold on;
plot(mag_pos,      gyro_pos,      'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Data +');
plot(mag_neg,      gyro_neg,      'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Data -');
plot(xq_pos,       yq_pos,        'r-', 'LineWidth', 2, 'DisplayName', 'Linear fit +');
plot(-xq_neg,     -yq_neg,        'm-', 'LineWidth', 2, 'DisplayName', 'Poly fit -');
grid on;
xlabel('mag'); ylabel('gyro');
title('Mag to Gyro mapping');
legend('Location', 'northwest');

% ── R² for both ───────────────────────────────────────────────────────
y_pos_fit = polyval(p_pos, mag_pos);
ss_res = sum((gyro_pos - y_pos_fit).^2);
ss_tot = sum((gyro_pos - mean(gyro_pos)).^2);
fprintf('Positive R² = %.6f\n', 1 - ss_res/ss_tot);

y_neg_fit = polyval(p_neg, (mag_neg_abs - mu_neg(1)) / mu_neg(2));
ss_res = sum((gyro_neg_abs - y_neg_fit).^2);
ss_tot = sum((gyro_neg_abs - mean(gyro_neg_abs)).^2);
fprintf('Negative R² = %.6f\n', 1 - ss_res/ss_tot);

fprintf('Positive linear: gyro = %.8f * mag + %.8f\n', p_pos(1), p_pos(2));
fprintf('Negative poly:   '); fprintf('%.8f ', p_neg); fprintf('\n');
fprintf('Negative mu:     %.8f %.8f\n', mu_neg(1), mu_neg(2));