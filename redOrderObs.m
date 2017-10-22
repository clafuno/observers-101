
%% State-space system
A=[0.6 1 0 2 0; 0 0.5 0.3 1.5 3; 0 0 0.8 0 3; 0 0 0 0.7 1;
 0 0 0 0 0.4];
B=[0 0; 0 0; 1 0; 0 0; 0 1];
C=[1 0 0 0 0 ; 0 1 0 0 0 ];
D=[0 0; 0 0];
Ts = 1; %seconds

sys = ss(A,B,C,D,Ts);
n = size(A,1); % Número de estados (rows of A)
m = size(B,2); % Número de entradas (columns of B)
p = size(C,1); % Número de salidas (rows of C)
rank(obsv(sys)) % observabilidad del sistema

x0 = [0 0 0 0 0]; % Estado inicial del sistema


%% Reduced Observer design

% "Separate" matrices

A11 = A(1:2,1:2); A12 = A(1:2,3:5);
A21 = A(3:5,1:2); A22 = A(3:5,3:5);
B1 = B(1:2,:); B2 = B(3:5,:);

p_ob_red = eig(A22)*0.5;
L_red = place(A22',A12',p_ob_red)'

Aob = A22 -L_red*A12;
Bob = [B2-L_red*B1 A21-L_red*A11+(A22-L_red*A12)*L_red];
Cob = eye(n-p);
Dob = [zeros(n-p,m) L_red];

sys_ob_red = ss(Aob,Bob,Cob,Dob,Ts)


%% Observer Simulink Simulation

% Parameters
Tsim = 10;
step1 = 1;
step2 = 2;
time = 0:Ts:Tsim;

x0_Obs = [5 5 5]; % Estado inicial del sistema

% Launch simulation and plot results
sim('sysObs')

% figure
% subplot(211)
% plot(time, simout(:,8)) %u1
% hold on; plot(time, simout(:,9)) %u2
% grid on; legend('u1','u2')
% subplot(212)
% plot(time, simout(:,1)) %y1
% hold on; plot(time, simout(:,2)) %y2
% grid on; legend('y1','y2')
% xlabel('Time(seconds)')


figure
subplot(311)
plot(time, simout(:,5)) %x3
hold on; plot(time, simoutOb(:,1)) %x3_est
grid on; legend('x3','x3-est')
subplot(312)
plot(time, simout(:,6)) %x4
hold on; plot(time, simoutOb(:,2)) %x4_est
grid on; legend('x4','x4-est')
subplot(313)
plot(time, simout(:,7)) %x5
hold on; plot(time, simoutOb(:,3)) %x5_est
grid on; legend('x5','x5-est')
xlabel('Time(seconds)')


%% Matlab reduced observer simulation

U = zeros(Tsim+1,2);
U(:,1) = step1;
U(:,2) = step2;
U(1,:) = 0;

x0_Obs = [-5 -5 -5]; % Estado inicial del sistema

% system simulation 
[Y,T,X] = lsim(sys,U,time,x0);

Uob = [U Y];
[X_est,T,Z] = lsim(sys_ob_red,Uob,time,x0_Obs);

% figure
% subplot(211)
% plot(time, U(:,1)) %u1
% hold on; plot(time, U(:,2)) %u2
% grid on; legend('u1','u2')
% subplot(212)
% plot(time, Y(:,1)) %y1
% hold on; plot(time, Y(:,2)) %y2
% grid on; legend('y1','y2')
% xlabel('Time(seconds)')


figure
subplot(311)
plot(time, X(:,3)) %x3
hold on; plot(time, X_est(:,1)) %x3_est
grid on; legend('x3','x3-est')
subplot(312)
plot(time, X(:,4)) %x4
hold on; plot(time, X_est(:,2)) %x4_est
grid on; legend('x4','x4-est')
subplot(313)
plot(time, X(:,5)) %x5
hold on; plot(time, X_est(:,3)) %x5_est
grid on; legend('x5','x5-est')
xlabel('Time(seconds)')
