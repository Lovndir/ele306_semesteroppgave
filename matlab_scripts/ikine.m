clear; clc;
%close all;
%% DH parameters for the robot arms
% Bruker alle verdier fra UR5 bortsett fra 'a'
% Bruker 'a' verdiene fra UR10

L1(1) = Link([ 0 0.089459 0 1.5708 ]);
L1(2) = Link([ 0 0 -0.68 0 ]);
L1(3) = Link([ 0 0 -0.66 0 ]);
L1(4) = Link([ 0 0.10915 0 1.5708 ]);
L1(5) = Link([ 0 0.09465 0 -1.5708 ]);
L1(6) = Link([ 0 0.0823 0 0 ]);

L2(1) = Link([ 0 0.089459 0 1.5708 ]);
L2(2) = Link([ 0 0 -0.68 0 ]);
L2(3) = Link([ 0 0 -0.66 0 ]);
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
%% Lokasjonar
% Lokasjon av interesse
SteikepanneHandtak = SE3(0.6, 0.4, 0.1) * SE3.Rx(0);
Steikepanne = SE3(0.75, 0.5, 0.1) * SE3.Rx(0);
TreFingra = SE3(0.3, 0.35, 0.1) * SE3.Rx(0);
Ingredienser = SE3(0.45, 0.5, 0.15) * SE3.Rx(pi);
Bowl = SE3(0.75, 0.2, 0.1) * SE3.Rx(0);
Smasher = SE3(1.1, 0.25, 0.1) * SE3.Rx(0);
Paddle = SE3(1.1, 0.35, 0.1) * SE3.Rx(0);
Kutter = SE3(1.1, 0.45, 0.1) * SE3.Rx(0);
% Lokasjon høgare oppe for bedre overganger
SteikepanneHandtakOvenfra = SE3(0.6, 0.4, 0.4) * SE3.Rx(0);
SteikepanneOvenfra = SE3(0.75, 0.5, 0.4) * SE3.Rx(0);
TreFingraOvenfra = SE3(0.3, 0.35, 0.4) * SE3.Rx(0);
IngredienserOvenfra = SE3(0.45, 0.5, 0.4) * SE3.Rx(pi);
BowlOvenfra = SE3(0.75, 0.2, 0.4) * SE3.Rx(0);
SmasherOvenfra = SE3(1.1, 0.25, 0.4) * SE3.Rx(0);
PaddleOvenfra = SE3(1.1, 0.35, 0.4) * SE3.Rx(0);
KutterOvenfra = SE3(1.1, 0.45, 0.4) * SE3.Rx(0);

t= [0:0.1:1]';
%% Arm1
% Vinkler for Arm1 på spesifikke lokasjoner
Smasher_AnglesArm1 = Arm1.ikine(Smasher);
Paddle_AnglesArm1 = Arm1.ikine(Paddle);
Steikepanne_AnglesArm1 = Arm1.ikine(Steikepanne);
Ingredienser_AnglesArm1 = Arm1.ikine(Ingredienser);
Bowl_AnglesArm1 = Arm1.ikine(Bowl);
Kutter_AnglesArm1 = Arm1.ikine(Kutter);

SmasherOvenfra_AnglesArm1 = Arm1.ikine(SmasherOvenfra);
PaddleOvenfra_AnglesArm1 = Arm1.ikine(PaddleOvenfra);
SteikepanneOvenfra_AnglesArm1 = Arm1.ikine(SteikepanneOvenfra);
IngredienserOvenfra_AnglesArm1 = Arm1.ikine(IngredienserOvenfra);
BowlOvenfra_AnglesArm1 = Arm1.ikine(BowlOvenfra);
KutterOvenfra_AnglesArm1 = Arm1.ikine(KutterOvenfra);

qo1_1 = jtraj(Smasher_AnglesArm1, Paddle_AnglesArm1, t);
qj_Arm1_PaddleTilPaddleOvenFra = jtraj(Paddle_AnglesArm1, PaddleOvenfra_AnglesArm1, t);
qj_Arm1_PaddleOvenFraTilSteikepanneOvenFra = jtraj(PaddleOvenfra_AnglesArm1, SteikepanneOvenfra_AnglesArm1, t);
qj_Arm1_SteikepanneOvenFraTilSteikepanne = jtraj(SteikepanneOvenfra_AnglesArm1, Steikepanne_AnglesArm1, t);


Ts1_1 = ctraj(Paddle, PaddleOvenfra, length(t));
Ts1_2 = ctraj(PaddleOvenfra, SteikepanneOvenfra, length(t));
Ts1_3 = ctraj(SteikepanneOvenfra, Steikepanne, length(t));

qc_Arm1_PaddleTilPaddleOvenFra = Arm1.ikine(Ts1_1);
qc_Arm1_PaddleOvenFraTilSteikepanneOvenFra = Arm1.ikine(Ts1_2);
qc_Arm1_SteikepanneOvenFraTilSteikepanne = Arm1.ikine(Ts1_3);

%% Arm2
% Vinkler for Arm2 på spesifikke lokasjoner
SteikepanneHandtak_AnglesArm2 = Arm2.ikine(SteikepanneHandtak);
TreFingra_AnglesArm2 = Arm2.ikine(TreFingra);
Ingredienser_AnglesArm2 = Arm2.ikine(Ingredienser);
Bowl_AnglesArm2 = Arm2.ikine(Bowl);

SteikepanneHandtakOvenfra_AnglesArm2 = Arm2.ikine(SteikepanneHandtakOvenfra);
TreFingraOvenfra_AnglesArm2 = Arm2.ikine(TreFingraOvenfra);
IngredienserOvenfra_AnglesArm2 = Arm2.ikine(IngredienserOvenfra);
BowlOvenfra_AnglesArm2 = Arm2.ikine(BowlOvenfra);

qj_Arm2_TreFingraTilTreFingraOvenFra = jtraj(TreFingra_AnglesArm2, TreFingraOvenfra_AnglesArm2, t);
qj_Arm2_TreFingraOvenFraTilIngredienserOvenFra = jtraj(TreFingraOvenfra_AnglesArm2, IngredienserOvenfra_AnglesArm2, t);
qj_Arm2_IngredienserOvenFraTilIngredienser = jtraj(IngredienserOvenfra_AnglesArm2, Ingredienser_AnglesArm2, t);

Ts2_1 = ctraj(TreFingra, TreFingraOvenfra, length(t));
Ts2_2 = ctraj(TreFingraOvenfra, IngredienserOvenfra, length(t));
Ts2_3 = ctraj(IngredienserOvenfra, Ingredienser, length(t));

qc_Arm2_TreFingraTilTreFingraOvenFra = Arm2.ikine(Ts2_1);
qc_Arm2_TreFingraOvenFraTilIngredienserOvenFra = Arm2.ikine(Ts2_2);
qc_Arm2_IngredienserOvenFraTilIngredienser = Arm2.ikine(Ts2_3);


%% Inverse Kinematics Motion
axis([-0.25 1.75 -0.25 1 0 1.5]);
hold on
Arm1.plot(qc_Arm1_PaddleTilPaddleOvenFra, 'workspace', [0 1.5 0 0.7 0 1])
Arm2.plot(qj_Arm2_TreFingraTilTreFingraOvenFra)
Arm1.plot(qc_Arm1_PaddleOvenFraTilSteikepanneOvenFra)
Arm2.plot(qj_Arm2_TreFingraOvenFraTilIngredienserOvenFra)
Arm1.plot(qc_Arm1_SteikepanneOvenFraTilSteikepanne)
Arm2.plot(qj_Arm2_IngredienserOvenFraTilIngredienser)


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
