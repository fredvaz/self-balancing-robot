
function [ Gth, Gv, Gth_info, Gv_info] = dynamic_system(rob_par, mot_par)
%% Dynamic System Function
%
% Compute the Process dynamics of a Self-Balancing robot
% 
% "l" is the distance between robot torso's gravity center and axle;
% "mw" is wheel's mass;
% "mc" is robot torso's mass;
% "Theta" is the intersection angle between robot's torso and the vertical upward direction;

% Robot and Motors Parameters
l=rob_par(1); mw=rob_par(2); mc=rob_par(3); r=rob_par(4); b=rob_par(5); g=rob_par(6);
La=mot_par(1); Ra=mot_par(2); Km=mot_par(3); Kb=mot_par(4); J=mot_par(5); B=mot_par(6);

%%                      System Transfer Function
s = tf('s');
% Simpliffication
Kb=0; 
La=0;
%mw = 0;

% Laplace Transform based on differential equations of the system
Ths = -(2*Km*s);
Vas = (r*(Ra + La*s)*(b*g - l*mw*s^3 + g*mc*s + g*mw*s - b*l*s^2));

Ths/Vas;

% Continuous-time zero/pole/gain model
Hs = zpk(  Ths/Vas );

zeros = cell2mat(Hs.Z)';
poles = cell2mat(Hs.P)';
gain = Hs.K;

Gth = Hs;
Gth_info = [zeros, poles, gain];

% Velocidade Linear
Xsl = (2*Km*(g - l*s^2));
Vas = -(r*(Ra + La*s)*(b*g + (mw + mc)*g*s - b*l*s^2 - l*mw*s^3));

Xsl/Vas;

% Continuous-time zero/pole/gain model
Hs = zpk(  Xsl/Vas );

zeros = cell2mat(Hs.Z)';
poles = cell2mat(Hs.P)';
gain = Hs.K;

Gv = Hs;
Gv_info = [zeros, poles, gain];

end
