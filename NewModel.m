clear; clc;
format long e;
h = 1e-3;

wn = 2*pi*5.53; %rad/s
ksi = 0.749; %-
alpha = 0.07528015464;
tau = 0.008; %actuator dynamics included
G = 39;
m2m1 = 3;

Vd = 0.507;
Vn = 0;

Anew = [[0 1 0]
    [-wn^2 -2*ksi*wn 1/tau*(1+m2m1)]
    [0 0 -1/tau]];

Bnew = [[0]
    [(G - (1+m2m1)/tau)]
    [(1/tau)]];

Cnew = [[1/40.33*0.9 0 0]
    [0 1/28.134*1.1 0]
    [0 0 0.026]];

Dnew = [0;0;0];

sys = ss(Anew,Bnew,Cnew,Dnew)

eig(sys)

%Simulation Duration
Tsim = 15;
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

[L,P,E] = lqe(Anew, G, Cnew, Qn, Rn)


Kalmansys = ss(Anew-L*Cnew,[Bnew L], eye(3), 0*[Bnew L])

Akg = Anew-L*Cnew;
Bkg = [Bnew L];
Ckg = eye(3);
Dkg = 0*[Bnew L];

Q = [[100 0 0]
    [0 100 0]
    [0 0 10]];

R = [[0.1]];

[Klqr,S,P] = lqr(sys,Q,R)

Klqr 
sim SimulinkModel.slx
