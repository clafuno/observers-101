
%% State-space system

A=[0.6 1 0 2 0; 0 0.5 0.3 1.5 3; 0 0 0.8 0 3; 0 0 0 0.7 1;
 0 0 0 0 0.4];
B=[0 0; 0 0; 1 0; 0 0; 0 1];
C=[1 0 0 0 0 ; 0 1 0 0 0 ];
D=[0 0; 0 0];

sys = ss(A,B,C,D,Ts);

%% Observer design

eig(A); % System poles
p_ob = eig(A)*0.5; % Poles of the observer should be two times faster than the system ones.
L = place(A',C',p_ob)' % Build L matrix (Observer gain)

% Build state-space observer
Aob = A - L*C;
Bob = [B L];
Cob = eye(n);
Dob = zeros(n,m+p);

sys_ob = ss(Aob,Bob,Cob,Dob,Ts)


%% Observer Matlab simulation

Ts = 1; % seconds
Tsim = 10; % seconds
time = 0:Ts:Tsim;

step1 = 1;
step2 = 2;
U = zeros(Tsim+1,2);
U(:,1) = step1;
U(:,2) = step2;
U(1,:) = 0;

x0 = [0 0 0 0 0];  % System initial state
x0_Obs = [10 10 10 10 10]; % Observer inital state

% System simulation 
[Y,T,X] = lsim(sys,U,time,x0);

% The input vector for the state observer is composed by the input and the ouput of the system.
Uob = [U Y];
% Observer simulation 
[X_est,T,Z] = lsim(sys_ob,Uob,time,x0_Obs);

% Plot results
figure
subplot(211)
plot(time, U(:,1)) %u1
hold on; plot(time, U(:,2)) %u2
grid on; legend('u1','u2')
subplot(212)
plot(time, Y(:,1)) %y1
hold on; plot(time, Y(:,2)) %y2
grid on; legend('y1','y2')
xlabel('Time(seconds)')


figure
subplot(511)
plot(time, X(:,1)) %x1
hold on; plot(time, Z(:,1)) %x1_est
grid on; legend('x1','x1-est')
subplot(512)
plot(time, X(:,2)) %x2
hold on; plot(time, Z(:,2)) %x2_est
grid on; legend('x2','x2-est')
subplot(513)
plot(time, X(:,3)) %x3
hold on; plot(time, Z(:,3)) %x3_est
grid on; legend('x3','x3-est')
subplot(514)
plot(time, X(:,4)) %x4
hold on; plot(time, Z(:,4)) %x4_est
grid on; legend('x4','x4-est')
subplot(515)
plot(time, X(:,5)) %x5
hold on; plot(time, Z(:,5)) %x5_est
grid on; legend('x5','x5-est')
xlabel('Time(seconds)')

%% Observer Simulink Simulation

% Parameters
Tsim = 10;
step1 = 1;
step2 = 2;
time = 0:Ts:Tsim;

x0_Obs = [10 10 10 10 10]; % Observer initial state

% Launch simulation and plot results
sim('sysObs')

figure
subplot(211)
plot(time, simout(:,8)) %u1
hold on; plot(time, simout(:,9)) %u2
grid on; legend('u1','u2')
subplot(212)
plot(time, simout(:,1)) %y1
hold on; plot(time, simout(:,2)) %y2
grid on; legend('y1','y2')
xlabel('Time(seconds)')

figure
subplot(511)
plot(time, simout(:,3)) %x1
hold on; plot(time, simoutOb(:,1)) %x1_est
grid on; legend('x1','x1-est')
subplot(512)
plot(time, simout(:,4)) %x2
hold on; plot(time, simoutOb(:,2)) %x2_est
grid on; legend('x2','x2-est')
subplot(513)
plot(time, simout(:,5)) %x3
hold on; plot(time, simoutOb(:,3)) %x3_est
grid on; legend('x3','x3-est')
subplot(514)
plot(time, simout(:,6)) %x4
hold on; plot(time, simoutOb(:,4)) %x4_est
grid on; legend('x4','x4-est')
subplot(515)
plot(time, simout(:,7)) %x5
hold on; plot(time, simoutOb(:,5)) %x5_est
grid on; legend('x5','x5-est')
xlabel('Time(seconds)')



