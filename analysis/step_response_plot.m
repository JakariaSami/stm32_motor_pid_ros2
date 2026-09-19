clear;

% Loading PI data.
pi_data = readmatrix('step_response.csv');
t_pi   = (pi_data(:,1) - pi_data(1,1)) / 1000;
sp_pi  = pi_data(:,2);
spd_pi = pi_data(:,3);

% loading P_only data.
p_data = readmatrix('step_P_only.csv');
t_p   = (p_data(:,1) - p_data(1,1)) / 1000;
sp_p  = p_data(:,2);
spd_p = p_data(:,3);

pi_mask = (t_pi >= 5) & (t_pi <= 10);
p_mask  = (t_p  >= 5.4) & (t_p  <= 10);

t_pi = t_pi(pi_mask);  spd_pi = spd_pi(pi_mask);  sp_pi = sp_pi(pi_mask);
t_p  = t_p(p_mask);    spd_p  = spd_p(p_mask);     sp_p  = sp_p(p_mask);

t_pi = t_pi - t_pi(1);
t_p  = t_p  - t_p(1);

% plots
figure;
plot(t_pi, spd_pi, 'LineWidth', 1.5, 'DisplayName', 'PI control (with integral)');
hold on;
plot(t_p,  spd_p,  'LineWidth', 1.5, 'DisplayName', 'P-only control');
plot(t_pi, sp_pi, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Setpoint');
hold off;

xlabel('Time (s)');
ylabel('Speed (encoder counts / 10ms)');
title('P-only vs PI Control: Steady-State Error');
legend('Location', 'southeast');
grid on;