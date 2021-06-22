clear
clc
close all

addpath('Toolbox')
addpath('CAD files/Step')

%% DP12evo Kynematics Data

% Front Right
A_FR   = [1648.3468978, -273.2057494, 303.4412329];       % [mm]
B_FR   = [1406.7643420, -273.3271603, 309.8895136];       % [mm]
UBJ_FR = [1510.800,	    -527.7856000,	  350.000];       % [mm]

C_FR   = [1698.5758126, -262.9263855, 126.1527273];       % [mm]
D_FR   = [1442.2098770, -263.1811473, 150.8282069];       % [mm]
LBJ_FR = [1539.300,         -563.500,  147.453300];       % [mm]

TR_inner_FR = [1490, -266.3096696, 119.8719506];          % [mm]       
TR_outer_FR = [1490,     -553.400,     118.540];          % [mm]
    
WC_FR = [1550, -612.500, 235];                            % [mm]
CP_FR = [1550, -612.500,   0];                            % [mm]

% Front Left
A_FL   = [1648.3468978, 273.2057494, 303.4412329];        % [mm]
B_FL   = [1406.7643420, 273.3271603, 309.8895136];        % [mm]
UBJ_FL = [1510.800,	 527.7856000,	     350.000];        % [mm]

C_FL   = [1698.5758126, 262.9263855, 126.1527273];        % [mm]
D_FL   = [1442.2098770, 263.1811473, 150.8282069];        % [mm]
LBJ_FL = [1539.300,         563.500,  147.453300];        % [mm]

TR_inner_FL = [1490, 266.3096696, 119.8719506];           % [mm]       
TR_outer_FL = [1490,     553.400,     118.540];           % [mm]
    
WC_FL = [1550, 612.500, 235];                             % [mm]
CP_FL = [1550, 612.500,   0];                             % [mm]

% Rear Right
A_RR   = [125.1611741,  -318.3389843, 314.9022428];       % [mm]     
B_RR   = [-160.6422302, -318.2359593,  310.835013];       % [mm]     
UBJ_RR = [-25.600,          -530.800,     350.000];       % [mm]

C_RR   = [88.6055420,   -307.5960121, 143.1451759];       % [mm] 
D_RR   = [-169.5860049, -307.0958886, 119.9728276];       % [mm]
LBJ_RR = [-6.400,           -569.500,     130.400];       % [mm]        

TR_inner_RR = [1490, -266.3096697, 119.8719506];          % [mm]
TR_outer_RR = [1490,       -553.4,      118.54];          % [mm]

WC_RR  = [0, -612.500, 235];                              % [mm]
CP_RR  = [0, -612.500,   0];                              % [mm]

% Rear Left
A_RL   = [125.1611741,  318.3389843, 314.9022428];        % [mm]     
B_RL   = [-160.6422302, 318.2359593,  310.835013];        % [mm]     
UBJ_RL = [-25.600,          530.800,     350.000];        % [mm]

C_RL   = [88.6055420,    307.5960121, 143.1451759];       % [mm] 
D_RL   = [-169.5860049,  307.0958886, 119.9728276];       % [mm]
LBJ_RL = [-6.400,            569.500,     130.400];       % [mm] 

TR_inner_RL = [1490, 266.3096697, 119.8719506];           % [mm]
TR_outer_RL = [1490,       553.4,      118.54];           % [mm]
 
WC_RL  = [0, 612.500, 235];                               % [mm]
CP_RL  = [0, 612.500,   0];                               % [mm]

%% Geometry Definition

%Front Right
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

%Front Left
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

%Rear Right
P1_RR = biella_equivalente(A_RR,B_RR,UBJ_RR);          % upper wishbone rotation centre
P2_RR = biella_equivalente(C_RR,D_RR,LBJ_RR);          % lower wishbone rotation centre

x1_RR = (A_RR - B_RR)/len(A_RR - B_RR);
y1_RR = (P1_RR - UBJ_RR)/len(P1_RR - UBJ_RR);
z1_RR = cross(x1_RR,y1_RR);
R1_RR = [x1_RR' y1_RR' z1_RR'];                        % upper wishbone rotation matrix

x2_RR = (C_RR - D_RR)/len(C_RR - D_RR);
y2_RR = (P2_RR - LBJ_RR)/len(P2_RR - LBJ_RR);
z2_RR = cross(x2_RR,y2_RR);
R2_RR = [x2_RR' y2_RR' z2_RR'];                        % lower wishbone rotation matrix

%Rear Left
P1_RL = biella_equivalente(A_RL,B_RL,UBJ_RL);          % upper wishbone rotation centre
P2_RL = biella_equivalente(C_RL,D_RL,LBJ_RL);          % lower wishbone rotation centre

x1_RL = (A_RL - B_RL)/len(A_RL - B_RL);
y1_RL = (P1_RL - UBJ_RL)/len(P1_RL - UBJ_RL);
z1_RL = cross(x1_RL,y1_RL);
R1_RL = [x1_RL' y1_RL' z1_RL'];                        % upper wishbone rotation matrix

x2_RL = (C_RL - D_RL)/len(C_RL - D_RL);
y2_RL = (P2_RL - LBJ_RL)/len(P2_RL - LBJ_RL);
z2_RL = cross(x2_RL,y2_RL);
R2_RL = [x2_RL' y2_RL' z2_RL'];                        % lower wishbone rotation matrix

s = 15;                                                % wishbones thickness

upper_wishbone_FR = CrossSection(A_FR,B_FR,UBJ_FR,s);
lower_wishbone_FR = CrossSection(C_FR,D_FR,LBJ_FR,s);

upper_wishbone_FL = CrossSection(A_FL,B_FL,UBJ_FL,s);
lower_wishbone_FL = CrossSection(C_FL,D_FL,LBJ_FL,s);

upper_wishbone_RR = CrossSection(A_RR,B_RR,UBJ_RR,s);
lower_wishbone_RR = CrossSection(C_RR,D_RR,LBJ_RR,s);

upper_wishbone_RL = CrossSection(A_RL,B_RL,UBJ_RL,s);
lower_wishbone_RL = CrossSection(C_RL,D_RL,LBJ_RL,s);

%% Mass Properties

% chassis
m_c  = 30023.63;                                            % mass: [g]
cm_c = [911.57, 0, 260.74];                                 % centre of mass: [mm]
mi_c = [2317933090.87, 15977074850.40, 16428169415.52];     % moments of inertia: [g*mm^2]
pi_c = [928656.46, 834747850.50, -349501.53];               % products of inertia: [g*mm^2]

% front upper wishbone
m_uw_f  = 193.05;                                           % mass: [g]
cm_uw_f = [17.25, 125.92, 2.43];                            % centre of mass: [mm]
mi_uw_f = [1775300.23, 1096113.00, 2845850.13];             % moments of inertia: [g*mm^2]
pi_uw_f = [323172.35, -9799.59, -46907.96];                 % products of inertia: [g*mm^2]

% front lower wishbone
m_lw_f  = 223.41;                                           % mass: [g]
cm_lw_f = [150.33, 76.20, -0.06];                           % centre of mass: [mm]
mi_lw_f = [1942442.96, 2335489.06, 4262184.02];             % moments of inertia: [g*mm^2]
pi_lw_f = [551273.84, 1887.80, 970.14];                     % products of inertia: [g*mm^2]

% front tie rod
m_tr_f  = 151.93;                                           % mass: [g]
cm_tr_f = [10.1701820, 0, 0];                               % centre of mass: [mm]
mi_tr_f = [2148.00, 868877.33, 868908.38];                  % moments of inertia: [g*mm^2]
pi_tr_f = [0.05, 0.09, -70.42];                             % products of inertia: [g*mm^2]

% front upright 
m_w_f  = 7281.20;                                           % mass: [g]
cm_w_f = [-2.5200, -0.5600, 1.2300];                        % centre of mass: [mm]
mi_w_f = [132050599.85, 216341477.67, 127876348.30];        % moments of inertia: [g*mm^2]
pi_w_f = [-626347.37, -807884.16, 419158.52];               % products of inertia: [g*mm^2]