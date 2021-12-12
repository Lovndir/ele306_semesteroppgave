clear; clc;
%close all;
%% DH parameters for the robot arms
% Bruker alle verdier fra UR5 bortsett fra 'a'
% Bruker 'a' verdiene fra UR10

L1(1) = Link([ 0 0.089459 0 1.5708 ]);
L1(2) = Link([ 0 0 -0.612 0 ]);
L1(3) = Link([ 0 0 -0.5723 0 ]);
L1(4) = Link([ 0 0.10915 0 1.5708 ]);
L1(5) = Link([ 0 0.09465 0 -1.5708 ]);
L1(6) = Link([ 0 0.0823 0 0 ]);

L2(1) = Link([ 0 0.089459 0 1.5708 ]);
L2(2) = Link([ 0 0 -0.612 0 ]);
L2(3) = Link([ 0 0 -0.5723 0 ]);
L2(4) = Link([ 0 0.10915 0 1.5708 ]);
L2(5) = Link([ 0 0.09465 0 -1.5708 ]);
L2(6) = Link([ 0 0.0823 0 0 ]);

Arm1 = SerialLink(L1,'name','Arm 1');
Arm2 = SerialLink(L2,'name','Arm 2');

%% Translate and rotate the both robot base
%Arm1.qlim = [[-pi pi]; [-pi pi]; [-pi pi]; [-pi pi]; [-pi pi]; [-pi pi]]; 
%Arm2.qlim = [[-pi pi]; [-pi pi]; [-pi pi]; [-pi pi]; [-pi pi]; [-pi pi]]; 

T_base1 = transl(1, 0, 0.5) * trotx(-90);
T_base2 = transl(0.5, 0, 0.5) * trotx(-90);
Arm1.base = T_base1
Arm2.base = T_base2

%%

q1_0 = [deg2rad(45) -deg2rad(60) -deg2rad(60) 0 0 0]; % Start position Arm1

q2_0 = [-deg2rad(45) -deg2rad(60) -deg2rad(60) 0 0 0]; % Start position Arm2


%% Inverse Kinematics Verdiar
% Vinkler
%% Arm1
T1_1 = SE3(1.3, 0.5, 0.5) * SE3.Rx(pi);
T1_2 = SE3(1.3, 0.7, 0.2) * SE3.Rx(pi);

q1_1 = Arm1.ikine(T1_1);
q1_2 = Arm1.ikine(T1_2);


t= [0:0.1:1]';

qo1_1 = jtraj(q1_1, q1_2, t);

Ts1_1 = ctraj(T1_1, T1_2, length(t));

qc1_1 = Arm1.ikine(Ts1_1);

%% Arm2
T2_1 = SE3(0.2, 0.6, 0.05) * SE3.Rx(pi);
T2_2 = SE3(0.2, 0.6, 0.5) * SE3.Rx(pi);
T2_3 = SE3(0.6, 0.6, 0.5) * SE3.Rx(pi);
T2_4 = SE3(0.6, 0.5, 0.05) * SE3.Rx(pi);

q2_1 = Arm2.ikine(T2_1);
q2_2 = Arm2.ikine(T2_2);
q2_3 = Arm2.ikine(T2_3);
q2_4 = Arm2.ikine(T2_4);

qj2_1 = jtraj(q2_1, q2_2, t);
qj2_2 = jtraj(q2_2, q2_3, t);
qj2_3 = jtraj(q2_3, q2_4, t);

Ts2_1 = ctraj(T2_1, T2_2, length(t));
Ts2_2 = ctraj(T2_2, T2_3, length(t));
Ts2_3 = ctraj(T2_3, T2_4, length(t));

qc2_1 = Arm2.ikine(Ts2_1);
qc2_2 = Arm2.ikine(Ts2_2);
qc2_3 = Arm2.ikine(Ts2_3);


%% Inverse Kinematics Motion
axis([-0.25 1.75 -0.25 1 0 1.5]);
hold on
Arm1.plot(qc1_1, 'workspace', [0 1.5 0 0.7 0 1])
Arm2.plot(qc2_1)
Arm2.plot(qj2_2)
Arm2.plot(qc2_3)


hold off
%% Plots
% % Joint vs time
% figure
% plot(qo1)
% figure
% plot(qo1)
% 
% % Cartesian vs space
% figure
% plot(t, Ts1_1.transl)

% 
% figure
% Arm1.plot(qf1_1)
% Arm1.plot(qi)
