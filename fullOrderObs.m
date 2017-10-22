
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



%% Simulink Simulation

% Parameters
Tsim = 10;
step1 = 1;
step2 = 2;
time = 0:Ts:Tsim;

x0 = [0 0 0 0 0]; % Estado inicial del sistema

% Launch simulation and plot results
sim('sysSim')

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
xlabel('Time(seconds)')



%% Matlab simulation

U = zeros(Tsim+1,2);
U(:,1) = step1;
U(:,2) = step2;
U(1,:) = 0;

% Launch simulation and plot results
[Y,T,X] = lsim(sys,U,time,x0);

figure
subplot(311)
plot(time, U(:,1)) %u1
hold on; plot(time, U(:,2)) %u2
grid on; legend('u1','u2')
subplot(312)
plot(time, Y(:,1)) %y1
hold on; plot(time, Y(:,2)) %y2
grid on; legend('y1','y2')
subplot(313)
plot(time, X(:,1)) %x1
hold on; plot(time, X(:,2)) %x2
hold on; plot(time, X(:,3)) %x3
hold on; plot(time, X(:,4)) %x4
hold on; plot(time, X(:,5)) %x5
grid on; legend('x1','x2','x3','x4','x5')
xlabel('Time(seconds)')


%% Observer design

eig(A); % Polos del sistema
p_ob = eig(A)*0.5; % Polos del observador el doble de rápidos.
L = place(A',C',p_ob)'

Aob = A -L*C;
Bob = [B L];
Cob = eye(n);
Dob = zeros(n,m+p);

sys_ob = ss(Aob,Bob,Cob,Dob,Ts)


%% Observer Simulink Simulation

% Parameters
Tsim = 10;
step1 = 1;
step2 = 2;
time = 0:Ts:Tsim;

x0_Obs = [10 10 10 10 10]; % Estado inicial del sistema

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


%% Observer Matlab simulation

U = zeros(Tsim+1,2);
U(:,1) = step1;
U(:,2) = step2;
U(1,:) = 0;

x0_Obs = [10 10 10 10 10]; % Estado inicial del sistema

% system simulation 
[Y,T,X] = lsim(sys,U,time,x0);

Uob = [U Y];
[X_est,T,Z] = lsim(sys_ob,Uob,time,x0_Obs);

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


