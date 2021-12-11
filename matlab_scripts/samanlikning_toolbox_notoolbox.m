%% Samanlikna forward Kinematics - toolbox vs no toolbox

clear, close all, clc;
%% DH parameters for the robot arms
% Bruker alle verdier fra UR5 bortsett fra 'a'
% Bruker 'a' verdiene fra UR10

%% No toolbox

q1=pi/2; q2=-pi/6; q3=-pi/4; q4=pi/2; q5=1; q6=1;
d1=0.089459; d2=0; d3=0; d4=0.10915; d5=0.09465; d6=0.0823;
a1=0; a2=-0.612; a3=-0.5723; a4=0; a5=0; a6=0;
alpha1=1.5708; alpha2=0; alpha3=0; alpha4=1.5708; alpha5=-1.5708; alpha6=0;

Transf0til1 = ([cos(q1) (-sin(q1)*cos(alpha1)) (sin(q1)*sin(alpha1)) (a1*cos(q1)); ...
    sin(q1) (cos(q1)*cos(alpha1)) (-cos(q1)*sin(alpha1)) (a1*sin(q1)); ...
    0 sin(alpha1) cos(alpha1) d1; ...
    0 0 0 1;]);

Transf1til2 = ([cos(q2) (-sin(q2)*cos(alpha2)) (sin(q2)*sin(alpha2)) (a2*cos(q2)); ...
    sin(q2) (cos(q2)*cos(alpha2)) (-cos(q2)*sin(alpha2)) (a2*sin(q2)); ...
    0 sin(alpha2) cos(alpha2) d2; ...
    0 0 0 1;]);
Transf2til3 = ([cos(q3) (-sin(q3)*cos(alpha3)) (sin(q3)*sin(alpha3)) (a3*cos(q3)); ...
    sin(q3) (cos(q3)*cos(alpha3)) (-cos(q3)*sin(alpha3)) (a3*sin(q3)); ...
    0 sin(alpha3) cos(alpha3) d3; ...
    0 0 0 1;]);
Transf3til4 = ([cos(q4) (-sin(q4)*cos(alpha4)) (sin(q4)*sin(alpha4)) (a4*cos(q4)); ...
    sin(q4) (cos(q4)*cos(alpha4)) (-cos(q4)*sin(alpha4)) (a4*sin(q4)); ...
    0 sin(alpha4) cos(alpha4) d4; ...
    0 0 0 1;]);
Transf4til5 = ([cos(q5) (-sin(q5)*cos(alpha5)) (sin(q5)*sin(alpha5)) (a5*cos(q5)); ...
    sin(q5) (cos(q5)*cos(alpha5)) (-cos(q5)*sin(alpha5)) (a5*sin(q5)); ...
    0 sin(alpha5) cos(alpha5) d5; ...
    0 0 0 1;]);
Transf5til6 = ([cos(q6) (-sin(q6)*cos(alpha6)) (sin(q6)*sin(alpha6)) (a6*cos(q6)); ...
    sin(q6) (cos(q6)*cos(alpha6)) (-cos(q6)*sin(alpha6)) (a6*sin(q6)); ...
    0 sin(alpha6) cos(alpha6) d6; ...
    0 0 0 1;]);

No_Toolbox = Transf0til1*Transf1til2*Transf2til3*Transf3til4*Transf4til5*Transf5til6;

%% Toolbox


L1(1) = Link([ 0 0.089459 0 1.5708 ]);
L1(2) = Link([ 0 0 -0.612 0 ]);
L1(3) = Link([ 0 0 -0.5723 0 ]);
L1(4) = Link([ 0 0.10915 0 1.5708 ]);
L1(5) = Link([ 0 0.09465 0 -1.5708 ]);
L1(6) = Link([ 0 0.0823 0 0 ]);

Arm1 = SerialLink(L1,'name','Arm 1');
qf1_1 = [pi/2 -pi/6 -pi/4 pi/2 1 1]
%% Output

% Toolbox
Tf1 = Arm1.fkine(qf1_1)

% No toolbox
No_Toolbox