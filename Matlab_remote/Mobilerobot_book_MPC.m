clc;
clear;
close all;

Ts = 0.033; % Sampling time
t = 0:Ts:30; % Simulation time
q = [1.1; 0.8; 0]; % Initial robot pose
%% 입력해줘야 하는것 %%%
%%-----------------------%%
%% 초기위치 q
%{
Ts = 0.050
t = 



%}
%%
% Reference

size_factor = 2;

freq = 2*pi/30;

Ts = 0.033; % Sampling time
t = 0:Ts:32; % Simulation time
q = [0; 0; 0+0.6]; % Initial robot pose
% Reference

freq = 2*pi/30;

% yRef = 1.1 + 0.7*sin(freq*t); xRef = 0.9 + 0.7*sin(2*freq*t);
% dyRef = freq*0.7*cos(freq*t); dxRef = 2*freq*0.7*cos(2*freq*t);
% ddyRef =-freq^2*0.7*sin(freq*t); ddxRef =-4*freq^2*0.7*sin(2*freq*t);


xRef = 0.0 + size_factor*sin(2*freq*t); yRef = 0.0 + 2*size_factor*sin(freq*t);
dxRef = 2*freq*size_factor*cos(2*freq*t); dyRef = 2*size_factor*freq*cos(freq*t);
ddxRef =-4*freq^2*size_factor*sin(2*freq*t); ddyRef =-2*freq^2*size_factor*sin(freq*t);



qRef = [xRef; yRef; atan2(dyRef, dxRef)]; % Reference trajectory
vRef = sqrt(dxRef.^2+dyRef.^2);
wRef = (dxRef.*ddyRef-dyRef.*ddxRef)./(dxRef.^2+dyRef.^2);
uRef = [vRef; wRef]; % Reference inputs


% xRef = 0.0 + size_factor*sin(2*freq*t); yRef = 0.0 + 2*size_factor*sin(freq*t);
% dxRef = 2*freq*size_factor*cos(2*freq*t); dyRef = 2*size_factor*freq*cos(freq*t);
% ddxRef =-4*freq^2*size_factor*sin(2*freq*t); ddyRef =-2*freq^2*size_factor*sin(freq*t);
% qRef = [xRef; yRef; atan2(dyRef, dxRef)]; % Reference trajectory


% yRef = 1.1 + 0.7*sin(freq*t); xRef = 0.9 + 0.7*sin(2*freq*t);
% dyRef = freq*0.7*cos(freq*t); dxRef = 2*freq*0.7*cos(2*freq*t);
% ddyRef =-freq^2*0.7*sin(freq*t); ddxRef =-4*freq^2*0.7*sin(2*freq*t);
% qRef = [xRef; yRef; atan2(dyRef, dxRef)]; % Reference trajectory


q1_data = zeros(length(t),1);
q2_data = zeros(length(t),1);
q3_data_before = zeros(length(t),1);
q3_data_after = zeros(length(t),1);
u1_data = zeros(length(t),1);
u2_data = zeros(length(t),1);

%size_factor*sin(0.1*t), 2*size_factor*sin(0.05*t)



for k = 1:length(t)-4
    e=[cos(q(3)), sin(q(3)), 0; ...
        -sin(q(3)), cos(q(3)), 0; ...
        0, 0, 1]*(qRef(:,k) - q); % Error vector
    e(3) = wrapToPi(e(3)); % Correct angle

    A0 = [1, Ts*uRef(2,k), 0;-Ts*uRef(2,k), 1, Ts*uRef(1,k); 0,0,1];
    A1 = [1, Ts*uRef(2,k+1), 0;-Ts*uRef(2,k+1), 1, Ts*uRef(1,k+1); 0,0,1];
    A2 = [1, Ts*uRef(2,k+2), 0;-Ts*uRef(2,k+2), 1, Ts*uRef(1,k+2); 0,0,1];
    A3 = [1, Ts*uRef(2,k+3), 0;-Ts*uRef(2,k+3), 1, Ts*uRef(1,k+3); 0,0,1];
    A4 = [1, Ts*uRef(2,k+4), 0;-Ts*uRef(2,k+4), 1, Ts*uRef(1,k+4); 0,0,1];
    B = [Ts, 0; 0, 0; 0, Ts];
    C = eye(3);

    Z = zeros(3,2);
    Hm = [C*A0*B, Z, Z, Z; ...
        C*A0*A1*B, C*A0*B, Z, Z; ...
        C*A0*A1*A2*B, C*A0*A1*B, C*A0*B, Z; ...
        C*A0*A1*A2*A3*B, C*A0*A1*A2*B, C*A0*A1*B, C*A0*B];
    Fm = [C*A0*A1, C*A0*A1*A2, C*A0*A1*A2*A3, C*A0*A1*A2*A3*A4].';

    ar = 0.65;
    Ar = eye(3)*ar; % Reference error dynamics
    H = 0;
    Fr = [Ar^(H+1), Ar^(H+2), Ar^(H+3), Ar^(H+4)].';

    % Weight matrices
    Qt = diag(repmat([1; 40; 0.1], 4, 1));
    Rt = diag(repmat([0.001; 0.001], 4, 1));

    % Optimal control calculation
    KKgpc = (Hm.'*Qt*Hm + Rt)\(Hm.'*Qt*(-Fm));
    KK = KKgpc(1:2,:); % Take current control gains

    v = -KK*e;
    uF = [uRef(1,k)*cos(e(3)); uRef(2,k)];
    u = v + uF;

    vMAX = 1; wMAX = 15; % Max velocities
    if abs(u(1))>vMAX, u(1) = sign(u(1))*vMAX; end
    if abs(u(2))>wMAX, u(2) = sign(u(2))*wMAX; end
    
    % Robot motion simulation
    dq = [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)];
    noise = 0.000; % Set to experiment with noise (e.g. 0.001)
    q = q + Ts*dq + randn(3,1)*noise; % Euler integration
    q3_data_before(k) = q(3);
    q(3) = wrapToPi(q(3)); % Map orientation angle to [-pi, pi]
    
    q1_data(k) = q(1);
    q2_data(k) = q(2);
    q3_data_after(k) = q(3);

    u1_data(k) = u(1); % v값 u(1) 값임
    u2_data(k) = u(2); % w값 u(2) 값임 

    e1_data(k) = e(1); % e값 x에러 값임
    e2_data(k) = e(2); % e값 x에러 값임
    e3_data(k) = e(3); % e값 x에러 값임

    %disp(q(3))
    % hold on

end
plot(u1_data);
figure(2)
%scatter(q1_data,q2_data, 10,'.')
t = linspace(0,130,131);
plot(q1_data,q2_data,'b-')
hold on
plot(size_factor*sin(0.1*t), size_factor*2*sin(0.05*t),'k--')
title('Trajectory')
xlabel('X position(m)')
ylabel('Y position(m)')
xlim([-2.5 2.5]);
ylim([-5 5]);
axis equal
h=legend("MPC","Reference");    set(h,'fontsize',10);


figure(3)
plot(u2_data);

figure(4)
%plot(1:966,sqrt((e1_data.^2)+(e2_data.^2)))
plot(1:966,e1_data);

%{
Original
Ts = 0.033; % Sampling time
t = 0:Ts:30; % Simulation time
q = [1.1; 0.8; 0]; % Initial robot pose
% Reference
freq = 2*pi/30;
xRef = 1.1 + 0.7*sin(freq*t); yRef = 0.9 + 0.7*sin(2*freq*t);
dxRef = freq*0.7*cos(freq*t); dyRef = 2*freq*0.7*cos(2*freq*t);
ddxRef =-freq^2*0.7*sin(freq*t); ddyRef =-4*freq^2*0.7*sin(2*freq*t);
qRef = [xRef; yRef; atan2(dyRef, dxRef)]; % Reference trajectory
vRef = sqrt(dxRef.^2+dyRef.^2);
wRef = (dxRef.*ddyRef-dyRef.*ddxRef)./(dxRef.^2+dyRef.^2);
uRef = [vRef; wRef]; % Reference inputs



%}
