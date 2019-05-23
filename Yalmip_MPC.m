
clc;
clear all;
%% Parameter Setting:
m = 1575;
Iz = 2875;
lf = 1.2;
lr = 1.6;
Cf = 19000;
Cr = 33000;
Vx = 15;
%% Dynamic system
A = [-(2*Cf+2*Cr)/m/Vx 0 -Vx-(2*Cf*lf-2*Cr*lr)/m/Vx 0;
0 0 1 0;
-(2*Cf*lf-2*Cr*lr)/Iz/Vx 0 -(2*Cf*lf^2+2*Cr*lr^2)/Iz/Vx 0;
1 Vx 0 0];
B = [2*Cf/m 0 2*Cf*lf/Iz 0]';
C = [0 0 0 1; 0 1 0 0];
vehicle=ss(A,B,C,0);

%% Sampling Time and Simulation Time
T = 15;% simulation duration
Ts=0.1;% Sampling time
time = 0:0.1:T; % simulation time
% Discrete-time model
Gd = c2d(vehicle,Ts);
Ad = Gd.A;
Bd = Gd.B;
%% Load Trajectory Data and add YALMIP toolbox
load Ref.mat;

%% Define data for MPC controller
N = 5;
Q = [100 0; 0 100];
Qf = [100 0; 0 100];
R = 0.1;
X=[0;0;0;0];
%% Yalmip 
u = sdpvar(repmat(1,1,N),repmat(1,1,N)); %symbolic decision variables for Yalmip
r = sdpvar(repmat(2,1,N),repmat(1,1,N));
x = sdpvar(repmat(4,1,N+1),repmat(1,1,N+1));



%% Initializing MPC
constraints = [];
objective = 0;

for k = 1:N-1
    
objective = objective + (r{k}-C*x{k})'*Q*(r{k}-C*x{k})+u{k}'*R*u{k}; %objective function

constraints = [constraints, x{k+1} == Ad*x{k}+Bd*u{k}];
constraints = [constraints, -0.05 <= u{k+1}-u{k} <= 0.05];
constraints = [constraints, -0.1 <= u{k}<= 0.1];
constraints = [constraints, [-0.5;-0.5] <= C*x{k} - r{k} <= [0.5;0.5]];
end

objective = (C*x{N}-r{N})'*Qf*(C*x{N}-r{N})+objective; %objective function with penalized function

for j=1:1:150-N
    % Setup the optimization problem
    if j==1
   %% designing MPC controller with YALMIP toolbox
    Controller = optimizer(constraints,objective,[],{x{j},r{j},r{j+1},r{j+2},r{j+3},r{j+4}},u{j});% with current state,and next 5 steps with a single output
    end 
uout = Controller{{X(:,j),Yd(:,j),Yd(:,j+1),Yd(:,j+2),Yd(:,j+3),Yd(:,j+4)}};
X(:,j+1) = Ad*X(:,j)+Bd*uout;
Y(:,j+1)=C*X(:,j);
U(:,j)=uout;
end

%% Plotting
figure ;
plot(Y(1,1:145));
hold on;
plot(Yd(1,1:145))
legend('actual trajectory','reference')
 

