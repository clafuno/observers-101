
%% State-space system definition

% Discrete system defined by the matrices A, B, D and D.
A=[0.6 1 0 2 0; 0 0.5 0.3 1.5 3; 0 0 0.8 0 3; 0 0 0 0.7 1;
 0 0 0 0 0.4];
B=[0 0; 0 0; 1 0; 0 0; 0 1];
C=[1 0 0 0 0 ; 0 1 0 0 0 ];
D=[0 0; 0 0];
% System composed by 5 states, with 2 inputs and 2 outputs.

n = size(A,1); % Number of states (rows of A)
m = size(B,2); % Number of inputs (columns of B)
p = size(C,1); % Number of outputs (rows of C)
rank(obsv(sys)) % System observability (if observability matches the number of states, the system is fully observable)

% Build state-space system
sys = ss(A,B,C,D,Ts);

%% Simulation

% Time vector for simulation
Ts = 1; % seconds
Tsim = 10; % seconds
time = 0:Ts:Tsim;

% Definition of Input vector U. 
% Each one of the two inputs is an step of values step1 and step2.
step1 = 1;
step2 = 2;
U = zeros(Tsim+1,2);
U(:,1) = step1;
U(:,2) = step2;
U(1,:) = 0;

% System initial state
x0 = [0 0 0 0 0];

% Launch simulation
% Function lsim takes as input arguments:
% - the state-space system
% - the input(s) vector
% - the time vector
% - the system initial state
% The results of the function consists of
% - Y: the output of the system
% - X: the state(s) vector
[Y,T,X] = lsim(sys,U,time,x0);


% Plot results
figure
subplot(311) % INPUTS
plot(time, U(:,1)) %u1
hold on; plot(time, U(:,2)) %u2
grid on; legend('u1','u2')
subplot(312) % OUTPUTS
plot(time, Y(:,1)) %y1
hold on; plot(time, Y(:,2)) %y2
grid on; legend('y1','y2')
subplot(313) % STATES
plot(time, X(:,1)) %x1
hold on; plot(time, X(:,2)) %x2
hold on; plot(time, X(:,3)) %x3
hold on; plot(time, X(:,4)) %x4
hold on; plot(time, X(:,5)) %x5
grid on; legend('x1','x2','x3','x4','x5')

%% Simulink Simulation

% Parameters
Ts = 1; % seconds
Tsim = 10; % seconds
step1 = 1;
step2 = 2;
time = 0:Ts:Tsim;

x0 = [0 0 0 0 0]; % Estado inicial del sistema

% Launch simulation and plot results
sysSim

figure
subplot(311)
plot(time, simout(:,8)) %u1
hold on; plot(time, simout(:,9)) %u2
grid on; legend('u1','u2')
subplot(312)
plot(time, simout(:,1)) %y1
hold on; plot(time, simout(:,2)) %y2
grid on; legend('y1','y2')
subplot(313)
plot(time, simout(:,3)) %x1
hold on; plot(time, simout(:,4)) %x2
hold on; plot(time, simout(:,5)) %x3
hold on; plot(time, simout(:,6)) %x4
hold on; plot(time, simout(:,7)) %x5
grid on; legend('x1','x2','x3','x4','x5')






