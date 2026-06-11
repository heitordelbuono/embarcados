% Parametros da funcao de transferencia G(s)
m = 0.0027; % massa da bolinha
R = 0.02; % raio da bolinha
t = 0.00086; % espessura da bolinha
g = -9.8; % aceleração da gravidade
L = 0.095; % comprimento da metade da mesa
d = 0.03; % comprimento da haste conectada com o servo
J = 2*m/5*(R^2-(R-t)^2); % momento de inércia da bolinha
Td = 1; % constante derivativa
N = 100; % ordem do filtro derivativo
Kp = 1; % constante proporcional

s = tf('s');

G = -m*g*d/(L*(J/R^2+m)*s^2); % FT da planta

H = Kp*(1+Td*s/(Td/N*s+1)); % FT do controlador PID

GH = G*H;

[r, k] = rlocus(GH);
