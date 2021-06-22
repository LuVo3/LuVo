clear
clc
close all

addpath('Toolbox')
addpath('CAD files/Step')

%% DP12evo Kynematics Data

A_FR   = [1648.3468978, -273.2057494, 303.4412329];       % [mm]
B_FR   = [1406.7643420, -273.3271603, 309.8895136];       % [mm]
UBJ_FR = [1510.800,	 -527.7856000,	   350.000];          % [mm]

C_FR   = [1698.5758126, -262.9263855, 126.1527273];       % [mm]
D_FR   = [1442.2098770, -263.1811473, 150.8282069];       % [mm]
LBJ_FR = [1539.300,         -563.500,  147.453300];       % [mm]

TR_inner_FR = [1490, -266.3096696, 119.8719506];          % [mm]       
TR_outer_FR = [1490,     -553.400,     118.540];          % [mm]

PR_inner_FR = [1474.33, -191.715, 544.03];                 % [mm]
PR_outer_FR = [1510.8, -527.7856, 350];                    % [mm]

rocker_pivot_FR = [1520.385, -180, 550.794];                  % [mm]
rocker_axis_FR  = [1520.385, -165, 524.813];                  % [mm]
rocker_shock_FR = [1528.677, -117.078, 587.122];              % [mm]
    
WC_FR = [1550, -612.500, 235];                            % [mm]

A_FL   = [1648.3468978, 273.2057494, 303.4412329];        % [mm]
B_FL   = [1406.7643420, 273.3271603, 309.8895136];        % [mm]
UBJ_FL = [1510.800,	 527.7856000,	   350.000];          % [mm]

C_FL   = [1698.5758126, 262.9263855, 126.1527273];        % [mm]
D_FL   = [1442.2098770, 263.1811473, 150.8282069];        % [mm]
LBJ_FL = [1539.300,         563.500,  147.453300];        % [mm]

TR_inner_FL = [1490, 266.3096696, 119.8719506];           % [mm]       
TR_outer_FL = [1490,     553.400,     118.540];           % [mm]

PR_inner_FL = [1474.33, 191.715, 544.03];                 % [mm]
PR_outer_FL = [1474.33, 527.7856, 350];                    % [mm]

rocker_pivot_FL = [1474.33, 180, 550.794];                  % [mm]
rocker_axis_FL  = [1474.33, 165, 524.813];                  % [mm]
rocker_shock_FL = [1528.677, 117.078, 587.122];              % [mm]

WC_FL = [1550, 612.500, 235];                             % [mm]

%% Geometry Definition

P1_FR = biella_equivalente(A_FR,B_FR,UBJ_FR);          % upper wishbone rotation centre
P2_FR = biella_equivalente(C_FR,D_FR,LBJ_FR);          % lower wishbone rotation centre

x1_FR = (A_FR - B_FR)/len(A_FR - B_FR);
y1_FR = (P1_FR - UBJ_FR)/len(P1_FR - UBJ_FR);
z1_FR = cross(x1_FR,y1_FR);
R1_FR = [x1_FR' y1_FR' z1_FR'];                        % upper wishbone rotation matrix

x2_FR = (C_FR - D_FR)/len(C_FR - D_FR);
y2_FR = (P2_FR - LBJ_FR)/len(P2_FR - LBJ_FR);
z2_FR = cross(x2_FR,y2_FR);
R2_FR = [x2_FR' y2_FR' z2_FR'];                        % lower wishbone rotation matrix

P1_FL = biella_equivalente(A_FL,B_FL,UBJ_FL);          % upper wishbone rotation centre
P2_FL = biella_equivalente(C_FL,D_FL,LBJ_FL);          % lower wishbone rotation centre

x1_FL = (A_FL - B_FL)/len(A_FL - B_FL);
y1_FL = (P1_FL - UBJ_FL)/len(P1_FL - UBJ_FL);
z1_FL = cross(x1_FL,y1_FL);
R1_FL = [x1_FL' y1_FL' z1_FL'];                        % upper wishbone rotation matrix

x2_FL = (C_FL - D_FL)/len(C_FL - D_FL);
y2_FL = (P2_FL - LBJ_FL)/len(P2_FL - LBJ_FL);
z2_FL = cross(x2_FL,y2_FL);
R2_FL = [x2_FL' y2_FL' z2_FL'];                        % lower wishbone rotation matrix

s = 15;                                                % wishbones thickness

upper_wishbone_FR = CrossSection(A_FR,B_FR,UBJ_FR,s);
lower_wishbone_FR = CrossSection(C_FR,D_FR,LBJ_FR,s);

upper_wishbone_FL = CrossSection(A_FL,B_FL,UBJ_FL,s);
lower_wishbone_FL = CrossSection(C_FL,D_FL,LBJ_FL,s);

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