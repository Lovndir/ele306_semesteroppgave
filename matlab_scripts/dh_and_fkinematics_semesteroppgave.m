clear; close; clc;

syms q1_1 q2_1 q3_1 q4_1 q5_1 q6_1 
syms q1_2 q2_2 q3_2 q4_2 q5_2 q6_2 
syms d1_1 d2_1 d3_1 d4_1 d5_1 d6_1 
syms d1_2 d2_2 d3_2 d4_2 d5_2 d6_2 
syms a1_1 a2_1 a3_1 a4_1 a5_1 a6_1 
syms a1_2 a2_2 a3_2 a4_2 a5_2 a6_2 
syms alpha1_1 alpha2_1 alpha3_1 alpha4_1 alpha5_1 alpha6_1
syms alpha1_2 alpha2_2 alpha3_2 alpha4_2 alpha5_2 alpha6_2


L1(1) = Link([ q1_1 d1_1 a1_1 alpha1_1 ])
L1(2) = Link([ q2_1 d2_1 a2_1 alpha2_1 ])
L1(3) = Link([ q3_1 d3_1 a3_1 alpha3_1 ])
L1(4) = Link([ q4_1 d4_1 a4_1 alpha4_1 ])
L1(5) = Link([ q5_1 d5_1 a5_1 alpha5_1 ])
L1(6) = Link([ q6_1 d6_1 a6_1 alpha6_1 ])

L2(1) = Link([ q1_2 d1_2 a1_2 alpha1_2 ])
L2(2) = Link([ q2_2 d2_2 a2_2 alpha2_2 ])
L2(3) = Link([ q3_2 d3_2 a3_2 alpha3_2 ])
L2(4) = Link([ q4_2 d4_2 a4_2 alpha4_2 ])
L2(5) = Link([ q5_2 d5_2 a5_2 alpha5_2 ])
L2(6) = Link([ q6_2 d6_2 a6_2 alpha6_2 ])

Robot_1 = SerialLink(L1,'name','Robot Arm 1')
Robot_1.fkine([q1_1 q2_1 q3_1 q4_1 q5_1 q6_1])

Robot_2 = SerialLink(L2,'name','Robot Arm 2')
Robot_2.fkine([q1_2 q2_2 q3_2 q4_2 q5_2 q6_2])
