clear; clc;
format short e
h = 1e-3;

wn = 2*pi*5.53; %rad/s
ksi = 0.749; %-
alpha = 0.07528015464;
tau = 0.008; %actuator dynamics included
G = 39;
m2m1 = 3;

Vd = 0.507;
Vn = 0;

A = [[0 1 0]
    [-wn^2 -2*ksi*wn 1/tau*(1+m2m1)]
    [0 0 -1/tau]];

B = [[0]
    [(G - (1+m2m1)/tau)]
    [(1/tau)]];

C = [[1/40.33*0.9 0 0]
    [0 1/28.134*1.1 0]
    [0 0 0.026]];

%[[1/40.33*0.9 0 0]
    %[0 1/28.134*1.1 0]
    %[0 0 0.026]];

D = [0;0;0];

sys = ss(A,B,C,D);

%Weighting functions:
%Sensitivity Weight: W_S, we'd like disturbance rejection at higher
%unmodelled freqs (low-pass filter)
W_S = makeweight(1.2,[wn,0.5],0.1);

%Complementary Sensitivity Weight: W_T, robustness to high-frequency
%uncertainty
W_T = makeweight(0.01,[500,1],5);

%Control Effort Weight: W_U, we'd like minimal/less control effort for same
%stabilising result, so less actuator usage at high frequencies, less
%abrupt
W_U = makeweight(0.1,[200,1],10);

P = augw(sys, W_S, W_U, W_T);

nmeas = 3;
ncont = 1;
[K, CL, gamma] = hinfsyn(P, nmeas, ncont);

gamma

figure
sigma(CL(1:3,:))
title('W\_S * S')
grid on

figure
sigma(CL(4,:))
title('W\_U * KS')
grid on

figure
sigma(CL(5:7,:))
title('W\_T * T')
grid on


% Check closed-loop poles are stable
pole(CL)

% Simulate disturbance response
t_sim = 0:1e-3:10;
d = zeros(size(t_sim)); 
d(1000:1050) = 1; % pulse disturbance at t=1s
figure
lsim(CL, [d;d;d], t_sim)

[Ak, Bk, Ck, Dk] = ssdata(K)

%Simulation Duration
Tsim = 10;
%Time Vector 
t = 0:h:Tsim;
amplitude = 1;
omega = 20;
phase = 0;
u = amplitude*sin(omega*t+phase);
w = Vd*randn(size(t));
n = Vn*randn(size(t));

simin = [t.' u.'];
simin1 = [t.' w.'];
simin2 = [t.' n.'];