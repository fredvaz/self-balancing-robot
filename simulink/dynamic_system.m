
function [ Hs, zeros, poles, gain ] = dynamic_system(l, m, mp, I, g)
%% Dynamic System Function
%
% Compute the dynamics of a Self-Balancing robot
%
% input [ u ]  is the control/command;
% output [ alfa ] is the response angle; 
%   
% "l" is the distance between robot torso's gravity center and axle;
% "m" is wheel's mass;
% "mp" is robot torso's mass;
% "alfa" is the intersection angle between robot's torso and the vertical upward direction;


%%                      System Transfer Function
s = tf('s');

% Laplace Transform based on differential equations of the system
As = ( mp * l )/( (2*m + mp)*(I + mp*l^2) - (mp*l)^2  );

Us = s^2 - ( (2*m+mp)*(mp*g*l) )/( (2*m + mp)*(I + mp*l^2) - (mp*l)^2 );

% Continuous-time zero/pole/gain model
Hs = zpk( As/Us );

zeros = cell2mat(Hs.Z)';
poles = cell2mat(Hs.P)';
gain = Hs.K;


end
