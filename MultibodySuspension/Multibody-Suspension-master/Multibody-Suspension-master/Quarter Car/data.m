clear
clc
close all

addpath('Toolbox')
addpath('CAD files/Step')

%% DP12evo Kynematics Data

A   = [1648.3468978, -273.2057494, 303.4412329];        % [mm]
B   = [1406.7643420, -273.3271603, 309.8895136];        % [mm]
UBJ = [1510.800,	 -527.7856000,	   350.000];        % [mm]

C   = [1698.5758126, -262.9263855, 126.1527273];        % [mm]
D   = [1442.2098770, -263.1811473, 150.8282069];        % [mm]
LBJ = [1539.300,         -563.500,  147.453300];        % [mm]

tie_in  = [1490, -266.3096696, 119.8719506];            % [mm]       
tie_out = [1490,     -553.400,     118.540];            % [mm]
    
WC = [1550, -612.500, 235];                             % [mm]

%% Geometry Definition

P1 = biella_equivalente(A,B,UBJ);          % upper wishbone rotation centre
P2 = biella_equivalente(C,D,LBJ);          % lower wishbone rotation centre

x1 = (A - B)/len(A - B);
y1 = (P1 - UBJ)/len(P1 - UBJ);
z1 = cross(x1,y1);
R1 = [x1' y1' z1'];                        % upper wishbone rotation matrix

x2 = (C - D)/len(C - D);
y2 = (P2 - LBJ)/len(P2 - LBJ);
z2 = cross(x2,y2);
R2 = [x2' y2' z2'];                        % lower wishbone rotation matrix

s = 15;                                    % wishbones thickness
upper_wishbone = CrossSection(A,B,UBJ,s);
lower_wishbone = CrossSection(C,D,LBJ,s);

%% Mass Properties

% chassis
m_c  = 30023.63;                                            % mass: [g]
cm_c = [911.57, 0, 260.74];                                 % centre of mass: [mm]
mi_c = [2317933090.87, 15977074850.40, 16428169415.52];     % moments of inertia: [g*mm^2]
pi_c = [928656.46, 834747850.50, -349501.53];               % products of inertia: [g*mm^2]

% upper wishbone
m_uw  = 193.05;                                             % mass: [g]
cm_uw = [17.25, 125.92, 2.43];                              % centre of mass: [mm]
mi_uw = [1775300.23, 1096113.00, 2845850.13];               % moments of inertia: [g*mm^2]
pi_uw = [323172.35, -9799.59, -46907.96];                   % products of inertia: [g*mm^2]

% lower wishbone
m_lw  = 223.41;                                             % mass: [g]
cm_lw = [150.33, 76.20, -0.06];                             % centre of mass: [mm]
mi_lw = [1942442.96, 2335489.06, 4262184.02];               % moments of inertia: [g*mm^2]
pi_lw = [551273.84, 1887.80, 970.14];                       % products of inertia: [g*mm^2]

% tie rod
m_tr  = 151.93;                                             % mass: [g]
cm_tr = [10.1701820, 0, 0];                                 % centre of mass: [mm]
mi_tr = [2148.00, 868877.33, 868908.38];                    % moments of inertia: [g*mm^2]
pi_tr = [0.05, 0.09, -70.42];                               % products of inertia: [g*mm^2]

% wheel 
m_w  = 7281.20;                                             % mass: [g]
cm_w = [-2.5200, -0.5600, 1.2300];                          % centre of mass: [mm]
mi_w = [132050599.85, 216341477.67, 127876348.30];          % moments of inertia: [g*mm^2]
pi_w = [-626347.37, -807884.16, 419158.52];                 % products of inertia: [g*mm^2]