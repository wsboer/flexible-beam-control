clear; clc;
format short e;
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

sys = ss(A,B,C,D)

eig(sys)

%Simulation Duration
Tsim = 10;
%Time Vector 
t = 0:h:Tsim;
%Input Vector
amplitude = 1;
omega = 20;
phase = 0;
u = amplitude*sin(omega*t+phase);
w = Vd*randn(size(t));
n = Vn*randn(size(t));

simin = [t.' u.'];
simin1 = [t.' w.'];
simin2 = [t.' n.'];

y = lsim(sys,u+w,t);

combined = [t.' u.' y(:,1) y(:,2)];

writematrix(combined, 'NewModelDataVd2Vn0.csv')


figure
plot(t,y)
%ylim([-0.15,0.15])
legend("Deflection","Deflection Velocity", "Cart Velocity")

G = eye(3);

Qn = [[5.98e-8 0 0] 
    [0 2.35e-4 0]
    [0 0 1e-5]];

Rn = 1e-4 * eye(3);

[L,P,E] = lqe(A, G, C, Qn, Rn);

Kalmansys = ss(A-L*C,[B L], eye(3), 0*[B L])
eig(Kalmansys)

Ak = A-L*C;
Bk = [B L];
Ck = eye(3);
Dk = 0*[B L];

Q = [[40 0 0]
    [0 80 0]
    [0 0 40]];

R = [[0.1]];

[Klqr,S,P] = lqr(sys,Q,R)
L
Klqr
