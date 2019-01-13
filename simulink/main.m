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
% November 2018; Last revision: December-2018
%--------------------------------------------------------------------------
%
% Description:
%
%--------------------------------------------------------------------------
%
% References:
%
%--------------------------------------------------------------------------

%%                       System Plant Parameters - Paper

% Robot Parameters
l = 0.85;  % 0.85; %0.15;       % distance between robot chassis gravity center and axle
m = 70;    % 70; %0.05;         % wheel's mass
M = 50;    % 50; %0.5;          % robot chassis mass
r = 0.25;  % 0.25; %0.12;       % radius of the wheel's
b = 0.3;   % 0.3; %0.03;        % wheel's friction force
I = 1;                          % robot chassis rotational inertia
g = 9.8;                        % gravitational force

% Motors Parameters
La = 0.0015; % 0.0015, 0.0029 (mHa -> Ha) (La grande converge em curva suave, mas afecta a resposta demora mais a antigir o torque desejado)
Ra = 2.0;    % 2, 1.3;          % menor R, maior torque
Km = 0.1;    % 0.1, 0.05        % Motor Electric Constant (N.m)
Kb = 0.08;   % 0.08, 0.04       % Electromagnetic Loss
J = 0.005;   %
B = 0.05;    % 
N = 10;      % 10, 1

%Kt = 0.63/2;                   % Kt is the Motor Torque Constant (N�m/A) (holding torque)
%Km = Kt/srqt(Ra);
%Ia = T/Kt;                     % Ia is the armature current of the machine (SI units A), T is the torque


% Compute Transfer Function on continuous time based on the Dynamics System
[ Hs, zeros, poles, k ] = dynamic_system(l, m, M, I, g);

p1 = poles(1); p2 = poles(2);

%%                       System Plant Parameters - B-robot

% Robot Parameters
l = 0.10;          % distance between robot chassis gravity center and axle
m = 0.05;          % wheel's mass
M = 1.2;           % robot chassis mass
r = 0.12;          % radius of the wheel's
b = 0.003;         % wheel's friction force
I = 0.0125;        % robot chassis rotational inertia
g = 9.8;           % gravitational force

% Motors Parameters - Nema 17 equivalent
La = 0.0029;       % mHa -> Ha (La grande converge em curva suave, mas afecta a resposta demora mais a antigir o torque desejado)
Ra = 1.3;          % menor R, maior torque
Km = 0.05;         % Motor Electric Constant (N.m)
Kb = 0.03;         % Electromagnetic Loss
J = 0.082;         %
B = 0.05;          % 
N = 1;             % no gear ratio: 1

max_torque = 0.45;

% ^ 12V, 2A ~ 0.45 Nm @ 150 rpm (500 steps/2)

%Kt = 0.63/2;                   % Kt is the Motor Torque Constant (N�m/A) (holding torque)
%Km = Kt/sqrt(Ra);
%Ia = T/Kt;                     % Ia is the armature current of the machine (SI units A), T is the torque


% Compute Transfer Function on continuous time based on the Dynamics System
[ Hs, zeros, poles, k ] = dynamic_system(l, m, M, I, g);

p1 = poles(1); p2 = poles(2);

%%                             Root Locus 

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

plot(T)
grid on


%%
clear all
clc
 
syms Va As s La Ra mw mc g l r b km kb

equation = Va == r*(La*s + Ra)*((mw + mc)*(g/s^2 - mc*l)*As*s^2 ...
               + b*(g/s^2 - mc*l)*As*s + mc*l*As*s^2)/km + (kb*(g/s^2 - mc*l)*s)/r;

% Coloca em ordem a As
x = collect(equation, As)

As_ = solve(x, As)
Va_ = solve(x, Va)

sol = simplify( As_ / Va_)

% Coloca em ordem a Va
%y = collect(equation, Va)

% x = solve(x, As/Va)
% x = simplify(x)
% x = isolate(equation, As)

%% 
clear all
clc

syms Va As s La Ra mw mc g l r b km kb


s = (km*r - kb*(g/s^2 - mc*l)*s)/(r^2*(La*s + Ra)*((mw + mc)*(g/s^2 - mc*l)*s^2 ...
                                 + b*(g/s^2 - mc*l)*s + mc*l*s^2))

s = simplify(s)

sol = subs(s, [La Ra mw mc g l r b km kb], [0.001 3.0 0.05 1.5 9.8 0.12 0.11 0.003 0.25 0.2])

sol = simplify(sol)






















