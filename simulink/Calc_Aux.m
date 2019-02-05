
%% Cálculos auxiliares para determinar os Processos
% source: https://www.mathworks.com/matlabcentral/answers/360262-solving-symbolic-matlab-equation-in-term-of-two-variables-x-va

% Coloca em ordem a Va
%y = collect(equation, Va)
% x = solve(x, As/Va)
% x = simplify(x)
% x = isolate(equation, As)
% x = expand()

%% Processo do Ângulo Pitch
clear all
clc

syms Th Va Xs s La Ra mw mc g l r b Km Kb
Kb=0; 
%La=0;

% Equação a partir das diferencias da cinemática
eqn = (mw + mc)*Xs*s^2 + b*Xs*s + mc*l*Th*s^2 == ...
      (2*Km/(r*(La*s + Ra))) * (Va - (Kb*Xs*s)/r);
   
% Substitui Xs - OK!
eqn = subs(eqn, Xs, (g/s^2 -l)*Th);

% Coloca a ordem Theta(s) - OK!                   
eqn = collect(eqn, Th);

syms F
eqn1 = subs(eqn, Th, F*Va);
Th/Va == simplify( solve(eqn1, F) )


%% Processo da Velocidade Linear
clear all
clc

syms Th Va Xs s La Ra mw mc g l r b Km Kb
Kb=0; 
%La=0;

% Equação a partir das diferencias da cinemática
eqn = (mw + mc)*Xs*s^2 + b*Xs*s + mc*l*Th*s^2 == ...
      (2*Km/(r*(La*s + Ra))) * (Va - (Kb*Xs*s)/r);
   
% Substitui Th - OK!
eqn = subs(eqn, Th, (s^2/(g-l*s^2))*Xs);

% Coloca a ordem X(s) - OK!                   
eqn = collect(eqn, Xs);

% X(s)s
eqn = (b + s*(mc + mw) + (l*mc*s^3)/(- l*s^2 + g))*Xs*s == (2*Km*Va)/(r*(Ra + La*s));

syms F
eqn1 = subs(eqn, Xs, F*Va);
Xs/Va == simplify( solve(eqn1, F) )

% Den
simplify( expand(r*s*(Ra + La*s)*(b + s*(mc + mw) + (l*mc*s^3)/(- l*s^2 + g))) )


%% Método análitico - Eq. Theta
syms s p Zeta wn a

eqn = (a*s + p)*(s^2 + 2*Zeta*wn*s + wn^2) == 0;

% Extended 
eqn = expand(eqn);

% Em ordem a s^2
eqn = collect(eqn, s^2);

% Em ordem a s
eqn = collect(eqn, s)

%%
clc

a_ = 1; %0.00078; %1.0;

% Pólo não dominante
p1 = -0.1; %-0.00089;

% Na margem da estabilidade
Kp_crit = 19.11;

% Frequência natural não amortecida: FIX
wn_ = sqrt(0.004586/-p1)

% Factor de amorticimento
Zeta_ = (0.1*Kp_crit - 1.911 - a_*wn_^2)/(2*p1*wn_) 

% Controlo Derivativo
Kd_ = (a_*2*Zeta_*wn_ + p1 - 4.68e-5)/0.1

% Controlo Propocional
Kp_ = (2*p1*Zeta_*wn_ + a_*wn_^2 + 1.911)/0.1


%% TESTE
clc 

% Factor de amorticimento
Zeta_ = 0.7;
% Frequência natural não amortecida: 0.2141
% Nota:Menor w -> mais rápida a resposta, do sistema a estabilizar
% No entanto esta é uma característica física do sistema: capacidade dos
% motores, sistema global em si (ou seja frequência natural do sistema)
%wn_ = 0.5; 

% Controlo Propocional
Kp_ = (2*p1*Zeta_*wn_ + a_*wn_^2 + 1.911)/0.1

% Controlo Derivativo
Kd_ = (a_*2*Zeta_*wn_ + p1 - 4.68e-5)/0.1

% Pólo não dominante - FIX!
%p1_ = ((0.1*Kp_crit - 1.911 - a_*sqrt(0.004586))/(2*Zeta_*sqrt(0.004586)))^2 


%% 
clc
syms Kp

eqn = 0.1*Kp - 1.911 == 2*p*Zeta*wn + a*wn^2;

eqn = solve(eqn, p); % CHECK!

eqn = p == (0.1*Kp - 1.911 - a*0.004586/p)/(2*Zeta*sqrt(0.004586/p))

% Substitui Wn
%eqn = subs(eqn, wn, sqrt(0.004586/p));
eqn = solve(eqn, p) % CHECK!

% Substitui Wn
%eqn = subs(eqn, [Zeta a Kp], [1.0 0.00078 19.11])



%% Método análitico - Eq. V
% Correr a secção PI do main
clc

syms s Kp_pi Ki_pi

Kp_pd = 19.11;
Kd_pd = 3.5855;

Gc_PD = Kp_pd + Kd_pd*s;
Gc_PI = Kp_pi + Ki_pi/s;

Gv = ( -12.821*(s-9.899)*(s+9.899) ) / ((s+49.53)*(s-49.47)*(s+0.0024));


eqn = 1 + Gc_PI*Gc_PD*Gv == 0;

% Eq. Extensa
eqn = collect(eqn, s)

%% Por passos
clc 

eqn_p1 = s^2 * (s+49.53)*(s-49.47)*(s+0.0024);
eqn_p2 = ( Kp_pi*s+Ki_pi)*(19.11*s+3.52*s^2)* (-12.821)*(s-9.899)*(s+9.899);

% Ordem a s
eqn_p1 = collect(eqn_p1, s)

% Ordem a s
eqn_p2 = collect(eqn_p2, s)




%% Problema 1.3

% Malha Fechada
Ha = feedback(0.1*Gth_z, 1); % feedback 1, dá igual a Gth_z
step(Ha);
rlocus(Ha); 


%% Problema 3.1 - Controlo PID - Utilize o 2º método de sintonização de Ziegler-Nichols 

% b) Estudo da estabilidade do sistema em função do ganho proporcional
% Através do método de Routh-Hurwitz, resulta que o ganho critico é: kp = 5

% c) Frequência de oscilação do sistema na margem de estabilidade

% Na linha de s^2 c7/ kp = 5 encontrar a raíz s, pelo que w = 1 rad/s

% d) Lugar das Raízes (LR)
% Confirmamos visualmente os 3 pólos: s = 0, s = -0.5 e s = -2
rlocus(Gth); 

%% Valores críticos
K_cr = 5;               % b) Ganho Kp na Margem da Estabilidade 
w_cr = 1;               % c) Frequência de oscilação na Margem da Estabilidade 
P_cr = 2 * pi/ w_cr;    % Pontos criticos ??

% Com base na Tabela de valores de sintonização em malha fechada de Zeigler Nichols:
% http://pages.mtu.edu/~tbco/cm416/zn.html

% Controlo PID
Kp_pid = 0.6 * K_cr;

Ti_pid = 0.5 * P_cr;

Ki_pid = Kp_pid / Ti_pid;

Td_pid = (1/8) * P_cr;

Kd_pid = Kp_pid * Td_pid;

% Controlo PID - Gc(s)
Gc_PID = Kp_pid + Ki_pid/s + Kd_pid * s

H_PID = zpk( feedback( Gc_PID * G, 1) ) % M2 = 1, sensor/feedback = 1

[z_PID, p_PID] = zpkdata(H_PID, 'v');


%% Problema 3.3

% b) Controlador Discreto PI - Gc(z) - Caso prático da aplicação num MCU
% Um comportamento de 2ª ordem significa que os sistema deve ter 2 Pólos
% não serve para o nosso caso?

% de modo a que o sistema completo em malha fechada exiba um comportamento
% típico de 2ª ordem com factor de amortecimento ζ = 0.7 e frequência natural não
% amortecida w

% Normalmente o Professor dá o comportamento desejado? Como determino que
% comportamento deve ter o meu sistema?

% c) Vamos obter o modelo discreto do processo sob controlo, tomando em
% consideração o ZOH que o precede. Tendo este modelo, podemos efectuar
% analiticamente o projecto do controlador PI
% ^ pode resolver a nossa situação!
% comportamento típico de 2ª ordem... meh! ir ao cpa4, ver como se encontra
% a estabilidade??




















