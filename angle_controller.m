% get the plant transfer funciton for turning 

close all
clear all


counts_to_dps = 8.75/1000;

raw = csvread('data/gyro_step_170_140.csv',2);
% avg_z = mean(csvread('data/avg_z.csv'));

time = raw(:,1)/(1e6);
voltage = raw(:,5);
omega_z = raw(:,4)*counts_to_dps;

plot(time, voltage);

V_tot = 5.02;
V_0 = V_tot*(170-140)/400;
omega_ss = 200-omega_z(1);

omega_z = omega_z*pi/180;

figure
plot(time, omega_z);
hold on

% zeta = 0.3;
% omega_n = 20;
% pole = 17;
a = 35;

% A = omega_ss/V_0;

% third order fitting


% A = (pole*omega_n^2)*omega_ss/V_0;
A = a*omega_ss/V_0;

% covnert A to rad/sec

A = A*pi/180;

s = tf('s');

% P = A/((s+pole)*(s^2 + 2*zeta*omega_n*s + omega_n^2));
P = A/(s+a);

% P = tf([1],[1 6 14 24]);

[y_sim, t_sim] = step(P, 1.4);

plot(t_sim + time(1), V_0*y_sim + omega_z(1));


figure()
rlocus(P);

z = 25;

C_star = (z + s)/s;

GH = C_star*P;

rlocus(GH);

t_s = 0.20;
s_d = 4/t_s;

K = 1/abs(evalfr(GH,-s_d));

k_sum = 1;
k_p = K/k_sum;
k_i = k_p*z;

% k_i = z*k_p;

CLTF = K*GH/(1 + K*GH);

[y_sim, t_sim] = step(CLTF);

figure
plot(t_sim, y_sim);

P_yaw = P;


% now make the whole CLTF

% calcualte values

WD = 0.01927;
SW = 0.00676;
SD = 0.08581;
D = 0.04704;

CR = WD + D/2;

L = SD - CR + SW/2;


% faster means more overshoot

s = tf('s');
U = 0.3;
P = P_yaw*U^2/(D*s^2);
H = (L*s+U)/U;

% PD control
% decreased because k_p was too big
z = 30;
C_star = (s+z);

% C = 1;

figure
rlocus(C_star*P*H)
sd = -37;
GH = C_star*P*H;

K = -1/evalfr(GH, sd);

k_sum_line = 1;
k_d_line = K/k_sum_line;
k_p_line = z*k_d_line;

% Gcl1 = minreal(Kp*P*H/(1+Kp*P*H))
% figure
% [y_sim, t_sim] = step(Gcl1)

C = k_sum_line*(k_d_line*s + k_p_line);

G = C*P;

G_cl = GH/(1 + GH);

[y_sim, t_sim] = step(G_cl);

figure

plot(t_sim, 0.0134*y_sim);

% plot what requeste















