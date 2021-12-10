clear all, close all, clc;
%% DH parameters for the robot arms
% Bruker alle verdier fra UR5 bortsett fra 'a'
% Bruker 'a' verdiene fra UR10

L1(1) = Link([ 0 0.089459 0 1.5708 ]);
L1(2) = Link([ 0 0 -0.612 0 ]);
L1(3) = Link([ 0 0 -0.5723 0 ]);
L1(4) = Link([ 0 0.10915 0 1.5708 ]);
L1(5) = Link([ 0 0.09465 0 -1.5708 ]);
L1(6) = Link([ 0 0.0823 0 0 ]);

L2(1) = Link([ 0 0.1273 0 1.5708 ]);
L2(2) = Link([ 0 0 -0.612 0 ]);
L2(3) = Link([ 0 0 -0.5723 0 ]);
L2(4) = Link([ 0 0.163941 0 1.5708 ]);
L2(5) = Link([ 0 0.1157 0 -1.5708 ]);
L2(6) = Link([ 0 0.0922 0 0 ]);

Arm1 = SerialLink(L1,'name','Arm 1');
Arm2 = SerialLink(L2,'name','Arm 2');

%% Translate and rotate the both robot base

T_base1 = transl(-1, 0, 0) * trotx(pi/2);
T_base2 = transl(1, 0, 0) * trotx(pi/2);
Arm1.base = T_base1
Arm2.base = T_base2

%% Need to set the joint limits to plot/teach

Arm1.qlim = [[-pi pi]; [0 0.5]; [-pi pi]; [1 1]; [0 0]; [0 1]]; 
Arm2.qlim = [[pi pi]; [0 0.5]; [-pi pi]; [-pi -pi]; [0.5 -0.5]; [1 0]];

%%

q0 = [0 0 0 0 0 0] % Start position

qf1_1 = [pi/2 -pi/6 -pi/4 pi/2 1 1]
qf2_1 = [0 -pi/2 pi/2 -pi/2 pi/2 0]
Tf1 = Arm1.fkine(qf1_1)
Tf2 = Arm2.fkine(qf2_1)
 
% t = 0:0.15:3
% Q1 = jtraj(q0,qf1_1,t)
% Q2 = jtraj(q0,qf2_1,t)
% Tr1 = fkine(Arm1,Q1)
% Tr2 = fkine(Arm2,Q2)


axis([-2 2 -2 2 -1 2]);
hold on
Arm1.plot(q0)
Arm2.plot(q0)
for i=0:0.1:1
    Arm1.plot(qf1_1*i)
    Arm2.plot(qf2_1*i)
    i;
    %pause(0.000001)
end

%% ?

% for i = 1:1:length(t);
%     T1 = Tr1(i);
%     trs = transl(T1);
%     T2 = Tr2(i);
%     trs = transl(T2);
%     xx(i) = trs(1);
%     yy(i) = trs(2);
%     zz(i) = trs(3);
%     xx1(i) = trs(1);
%     yy1(i) = trs(2);
%     zz1(i) = trs(3);
% end
% plot3(xx,yy,zz,'Color',[1 0 0],'LineWidth',2)
% plot3(xx1,yy1,zz1,'Color',[0 1 0],'LineWidth',2)

%%

qf1_2 = [0 -pi/4 pi/2 pi/2 0 0]
qf2_2 = [-pi/2 0 0 0 0 0]
for i=0.1:0.1:1
    Arm1.plot(qf1_1 + (qf1_2*i))
    Arm2.plot(qf2_1 + (qf2_2*i))
    i;
    %pause(0.000001)
end

%%

qf1_3 = [4*pi 0 0 0 0 0]
qf2_3 = [pi/4 pi/6 -pi/4 0 0 0]
for i=0.1:0.1:1
    Arm1.plot(qf1_1 + qf1_2 + (qf1_3*i))
    Arm2.plot(qf2_1 + qf2_2 + (qf2_3*i))
    i;
    %pause(0.000001)
end
