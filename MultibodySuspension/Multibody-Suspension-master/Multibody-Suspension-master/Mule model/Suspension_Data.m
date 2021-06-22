clear
clc
close all

addpath('Toolbox')
addpath('CAD files/Step')


A   = [143.677671943, -213.787196761, 94.541807923];
B   = [-143.677671943, -211.159316178, 119.544421531];
UBJ = [0, -436.406375232, 130.579433925];

C = [143.151454868, -232.337922880, -81.956561259];
D = [-143.151454868, -236.012472268, -116.917563343];
LBJ = [0, -458.108316336, -75.900743103];

UBJ_r = [0, -20, -105];
LBJ_r = [0, -20, 105];

tie_length = 233.858127964;
toe_link_hub = [-80, 20, 85];

P1 = biella_equivalente(A,B,UBJ);
P2 = biella_equivalente(C,D,LBJ);


x1 = (A - B)/len(A - B);
y1 = (P1 - UBJ)/len(P1 - UBJ);
z1 = cross(x1,y1);
R1 = [x1' y1' z1'];

% cambiato segno x2
x2 = (D - C)/len(D - C);
y2 = (P2 - LBJ)/len(P2 - LBJ);
z2 = cross(x2,y2);
R2 = [x2' y2' z2'];

v = (UBJ - LBJ)/len(UBJ - LBJ);
gamma = 180/pi*(acos(dot(v, [0,0,1])));

%[alpha,beta,gamma] = planeAngles(A,B,UBJ);