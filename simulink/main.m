clear all
close all
clc

format short 

%--------------------------------------------------------------------------
%
%                            Self-Balancing Robot
%
%--------------------------------------------------------------------------
% Authors: Frederico Vaz
%          Pedro Eugénio
%
% email: fredvaz8@gmail.com
%        pedroeugenio@gmail.com
%
% November 2018; Last revision: February-2019
%--------------------------------------------------------------------------
%
% Description:
%
%--------------------------------------------------------------------------
%
% References:
%
%--------------------------------------------------------------------------


%%                       System Plant Parameters - B-robot

% Robot Parameters
l = 0.10;          % distance between robot chassis gravity center and axle
mw = 0.05;         % wheel's mass
mc = 1.2;          % robot chassis mass
r = 0.12;          % radius of the wheel's
b = 0.003;         % wheel's friction force
g = 9.8;           % gravitational force

% Motors Parameters - Nema 17 equivalent
La = 0.0029;       % mHa -> Ha
Ra = 1.3;          % menor R, maior torque
Km = 0.05;         % Motor Electric Constant (N.m)
Kb = 0.03;         % Electromagnetic Loss
J = 0.082;         % Motor rotor inertia
B = 0.05;          % Electromagnetic loss

max_torque = 0.45;
% ^ 12V, 2A ~ 0.45 Nm @ 150 rpm (500 steps/2)

robot_par = [l mw mc r b g];
motor_par = [La Ra Km Kb J B];

% All 
initial_theta = deg2rad(-5);

% Sample Time
h = 10e-3;


%%                             Process's 

% Compute Transfer Function on continuous time based on the Dynamics System
[  Gth, Gv, Gth_info, Gv_info ] = dynamic_system(robot_par, motor_par);

% Sample Time
h = 10e-3; % 0.02 s

% Continuous to Discrete ZOH method:
Gth_z = c2d(Gth, h, 'zoh');

% Continuous to Discrete ZOH method:
Gv_z = c2d(Gv, h, 'zoh');


%% TEST - Resposta em S?
figure;
step(Gth);
figure;
step(Gv);


%%                    PD Controller Continuous Time
% Based on Problema 3.1, Problema 3.3

Kp_pd = 43.307;
Kd_pd = 5.282;

% Controlo PI - Gc(s)
s = tf('s');
Gc_PD = Kp_pd + Kd_pd*s;

% Eq.
eq_pd = 1 + Gc_PD*Gth;

% Closed Loop
Lth_PD = feedback(Gc_PD*Gth, 1);

[z_PD, p_PD] = zpkdata(Lth_PD, 'v');


%%                    PI Controller Continuous Time

Kp_pi = 0.1504;
Ki_pi = 0.1106;

% Controlo PI - Gc(s)
s = tf('s');
Gc_PI = Kp_pi + Ki_pi/s;

% Closed Loop
Lv_PI = feedback(Gc_PI*Gc_PD*Gv, 1); %*Gc_PD

[z_PI, p_PI] = zpkdata(Lv_PI, 'v');

% Resposta a Step
% figure;
% step(Lv_PI);


%%                     Root Locus Continuous Time

% Theta Process vs Theta Process with PD controller
figure;
% Open Loop
subplot(1,2,1)
rlocus(Gth);
title('Malha Aberta');
% Closed Loop
subplot(1,2,2);
rlocus(Lth_PD);
title('Malha Fechada'); 
%grid on;

%% Velocity Process vs Velocity Process with PI controller
figure;
% Open Loop
subplot(1,2,1)
rlocus(Gv);
title('Malha Aberta');
% Closed Loop
subplot(1,2,2)
rlocus(Lv_PI);
title('Malha Fechada');


%%                       Controller Parameters 

h = 20e-3;

Pertub = 1;
k_pertub = 1;
t_pertub = 5;

k_noise = 1;

% Real Time
% kp = 0.4; %0.32;
% ki = 0.0; 
% kd = 0.15; %0.05;

% Simulation
% kp = 25.0; %
% ki = 0.0; % 
% kd = 3.0; % 

% All 
initial_theta = deg2rad(-5); % 0.3;

kp = 4000.0; 
ki = 50.0;
kd = 1200.0;

%% NOTAS:

% Usar um Motor DC equivale a fazer o controlo por tens�o, na realidade
% corresponde ao controlo de um PWM do Driver

% O Stepper Motor exige um controlo por steps, onde a tens�o e corrente s�o
% constantes, produzindo um determinado torque a determinada velocidade
% rpms ou steps/s

% O torque define-se como Nm (Newton metro), que corresponde a for�a em
% Newton por um ponto de aplica��o a 1m. Ora o raio das rodas corresponde
% ao nosso ponto de aplica��o com a superficie. Portanto divide-se Nm por r
% correspondente ao radio das rodas para obter a for�a

% Quanta for�a (Nm) � necess�ria para contrabalancear o sistema? 
% Configurar/escolher motor para gerar um torque suficiente

% O controlo do motor do paper est� gerar enorme tens�o fora do normal, pois
% a exig�ncia de torque � grande mas funciona!

% Force = Torque � [Length � sin (Angle)], converts torque into force
% In the equation, Angle is the angle at which the force acts on the lever 
% arm, where 90 degrees signifies direct application.

% As vezes o controlador pode nunca conseguir a estabilidade devidos as 
% caracter�sticas dos motores/sem pot�ncia suficiente por exemplo
% nos 1� testes era a indut�nica grande que afectava em grande a rapidez 
% da resposta


%% Gráficos

%% Margem da Estabilidade K's:  
Kp_pd = 19.11;
Kd_pd = 3.5855; % Kd_pd  muito próximo de Kp_pd -> Instabilidade
Kp_pi = 0.1204; %0.1504; % Ligeira mudança em Kp < Ki instabilidade | Kp mt maior que Ki melhor 
Ki_pi = 0.1106; %0.1106; % Ki mt maior que 0.1106 instabilidade

sim 'simulations/PID_Simulation_System_Discrete.slx'

% Plot Theta
fig = figure;
plot(theta, 'r', 'LineWidth', 1.5);
grid on;
title('Theta');
xlabel('Tempo (s)')
ylabel('Theta (deg)')
axis([0 12 -15 15]);
saveas(fig,'img/Theta_margin.pdf');

% Plot of Velocity
fig = figure;
plot(v ,'b', 'LineWidth', 1.5);
grid on;
title('Velocidade');
xlabel('Tempo (s)')
ylabel('Velocidade (m/s)')
axis([0 12 -1 1]);
saveas(fig,'img/Velocity_margin.pdf');

% Plot of Position
fig = figure;
plot(x, 'g', 'LineWidth', 1.5);
grid on;
title('Posição');
xlabel('Tempo (s)')
ylabel('Posição (m)')
axis([0 12 -0.5 0.5]);
saveas(fig,'img/Position_margin.pdf');

% Plot of Torque
fig = figure;
plot(T, 'c', 'LineWidth', 1.5);
grid on;
title('Torque');
xlabel('Tempo (s)')
ylabel('Torque (N.m)')
axis([0 12 -0.5 0.5]);
saveas(fig,'img/Torque_margin.pdf');


%% Estabilidade K's: 
Kp_pd = 35.9188;
Kd_pd = 3.44961;
Kp_pi = 0.1504;
Ki_pi = 0.1106;
%%
sim 'simulations/PID_Simulation_System_Discrete.slx'

% Plot Theta
fig = figure;
plot(theta, 'r', 'LineWidth', 1.5);
grid on;
title('Theta');
xlabel('Tempo (s)')
ylabel('Theta (deg)')
axis([0 12 -8 3]);
saveas(fig,'img/Theta_stable.pdf');

% Plot of Velocity
fig = figure;
plot(v, 'b', 'LineWidth', 1.5);
grid on;
title('Velocidade');
xlabel('Tempo (s)')
ylabel('Velocidade (m/s)')
axis([0 12 -0.3 0.15]);
saveas(fig,'img/Velocity_stable.pdf');

% Plot of Position
fig = figure;
plot(x, 'g', 'LineWidth', 1.5);
grid on;
title('Posição');
xlabel('Tempo (s)')
ylabel('Posição (m)')
axis([0 12 -0.1 0.05]);
saveas(fig,'img/Position_stable.pdf');

% Plot of Torque
fig = figure;
plot(T, 'c', 'LineWidth', 1.5);
grid on;
title('Torque');
xlabel('Tempo (s)')
ylabel('Torque (N.m)')
axis([0 12 -0.5 0.5]);
saveas(fig,'img/Torque_stable.pdf');


%% Estabilidade K's with NOISE
Kp_pd = 35.9188;
Kd_pd = 3.44961;
Kp_pi = 0.1504;
Ki_pi = 0.1106;

sim 'simulations/PID_Simulation_System_Discrete.slx'

% Plot Theta
fig = figure;
plot(theta, 'r', 'LineWidth', 1.1);
grid on;
title('Theta');
xlabel('Tempo (s)')
ylabel('Theta (deg)')
axis([0 12 -8 3]);
saveas(fig,'img/Theta_noise.pdf');

% Plot of Velocity
fig = figure;
plot(v, 'b', 'LineWidth', 1.1);
grid on;
title('Velocidade');
xlabel('Tempo (s)')
ylabel('Velocidade (m/s)')
axis([0 12 -0.3 0.15]);
saveas(fig,'img/Velocity_noise.pdf');

% Plot of Position
fig = figure;
plot(x, 'g', 'LineWidth', 1.1);
grid on;
title('Posição');
xlabel('Tempo (s)')
ylabel('Posição (m)')
axis([0 12 -0.1 0.05]);
saveas(fig,'img/Position_noise.pdf');

% Plot of Torque
fig = figure;
plot(T, 'c', 'LineWidth', 1.1);
grid on;
title('Torque');
xlabel('Tempo (s)')
ylabel('Torque (N.m)')
axis([0 12 -0.5 0.5]);
saveas(fig,'img/Torque_noise.pdf');


%% Estabilidade K's: 
Kp_pd = 35.9188;
Kd_pd = 3.44961;
Kp_pi = 0.07;
Ki_pi = 0.025;

%% sim 'simulations/PID_Simulation_System_Discrete.slx'

% Plot Theta
fig = figure;
plot(theta, 'r'); %, 'LineWidth', 1.25
grid on;
title('Theta');
xlabel('Tempo (s)')
ylabel('Theta (deg)')
%axis([0 150 -30 10]);
axis([0 105 -30 10]);
%saveas(fig,'img/Theta_experiment.pdf');
saveas(fig,'img/Theta_best_experiment.pdf');

% Plot of Velocity
fig = figure;
plot(v, 'b'); %, 'LineWidth', 1.25
grid on;
title('Velocidade');
xlabel('Tempo (s)')
ylabel('Velocidade (steps/s)')
%axis([0 150 -40 40]);
axis([0 105 -40 40]);
%saveas(fig,'img/Velocity_experiment.pdf');
saveas(fig,'img/Velocity_best_experiment.pdf');


















