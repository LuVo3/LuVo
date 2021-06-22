 %% Script to calculate the roll center gain of the car getting data from simscape
...multibody
    
clear
clc
close all

data

addpath('Toolbox')
addpath('CAD files/Step')
set(0, 'DefaultFigureWindowStyle', 'docked')

%% Import data from simscape multibody

output = sim('Front_Axle_Roll_Center','ReturnWorkspaceOutputs','on');

Camber_angle     = (output.Camber.data)*180/pi;
Toe_angle        = (output.Toe.data)*180/pi;

DINAMIC_UBJ_Y_FR = -output.UBJ_FR_Y.data;
DINAMIC_UBJ_Z_FR = -output.UBJ_FR_Z.data;
DINAMIC_LBJ_Y_FR = -output.LBJ_FR_Y.data;
DINAMIC_LBJ_Z_FR = -output.LBJ_FR_Z.data;

DINAMIC_UBJ_Y_FL = -output.UBJ_FL_Y.data;
DINAMIC_UBJ_Z_FL = -output.UBJ_FL_Z.data;
DINAMIC_LBJ_Y_FL = -output.LBJ_FL_Y.data;
DINAMIC_LBJ_Z_FL = -output.LBJ_FL_Z.data;

DINAMIC_CP_L_Y   = -output.CP_L_Y.data;
DINAMIC_CP_L_Z   = output.CP_L_Z.data;
DINAMIC_CP_R_Y   = -output.CP_R_Y.data;
DINAMIC_CP_R_Z   = output.CP_R_Z.data;

%% roll center

x_axis_limit = 2000;
x_axis       = linspace(-x_axis_limit, x_axis_limit);


phi          = zeros(length(DINAMIC_UBJ_Y_FR), 1);
RC_FY        = zeros(length(DINAMIC_UBJ_Y_FR), 1);
RC_FZ        = zeros(length(DINAMIC_UBJ_Y_FR), 1);
RC_Front     = zeros(length(DINAMIC_UBJ_Y_FR), 3);

for ii = 1:length(DINAMIC_UBJ_Y_FR)

    %Front left IC
    coefficients_UpperLeft = polyfit([DINAMIC_UBJ_Y_FL(ii), P1_FL(2)]...       % coefficients of line through UBJ_FL and P1_FL
                             ,[DINAMIC_UBJ_Z_FL(ii), P1_FL(3)], 1);
    slope_Upper_FL         = coefficients_UpperLeft (1);                       % slope of line through UBJ_FL and P1_FL
    offset_Upper_FL        = coefficients_UpperLeft (2);                       % offset of line through UBJ_FL and P1_FL
    
    
    coefficients_LowerLeft = polyfit([DINAMIC_LBJ_Y_FL(ii), P2_FL(2)]...       % coefficients of line through LBJ_FL and P2_FL
                             , [DINAMIC_LBJ_Z_FL(ii), P2_FL(3)], 1);
    slope_Lower_FL         = coefficients_LowerLeft (1);                       % slope of line through LBJ_FL and P2_FL
    offset_Lower_FL        = coefficients_LowerLeft (2);                       % offset of line through LBJ_FL and P2_FL
    
    
    IC_FL_X                = (offset_Lower_FL - offset_Upper_FL)/...
                             (slope_Upper_FL - slope_Lower_FL);                % find the IC x point
    IC_FL_Y                = slope_Upper_FL * IC_FL_X + ...
                             offset_Upper_FL;                                  % find the IC x point
    
    IC_FL = [WC_FL(1), IC_FL_X, IC_FL_Y];
    
    
    %Front right IC
    coefficients_UpperRight = polyfit([DINAMIC_UBJ_Y_FR(ii), P1_FR(2)]...      % coefficients of line through UBJ_FR and P1_FR
                              , [DINAMIC_UBJ_Z_FR(ii), P1_FR(3)], 1);
    slope_Upper_FR          = coefficients_UpperRight (1);                      % slope of line through UBJ_FR and P1_FR
    offset_Upper_FR         = coefficients_UpperRight (2);                      % offset of line through UBJ_FR and P1_FR
    
    
    coefficients_LowerRight = polyfit([DINAMIC_LBJ_Y_FR(ii), P2_FR(2)]...       % coefficients of line through LBJ_FR and P2_FR
                              , [DINAMIC_LBJ_Z_FR(ii), P2_FR(3)], 1);
    slope_Lower_FR          = coefficients_LowerRight (1);                      % slope of line through LBJ_FR and P2_FR
    offset_Lower_FR         = coefficients_LowerRight (2);                      % offset of line through LBJ_FR and P2_FR
    
    
    IC_FR_X                = (offset_Lower_FR - offset_Upper_FR)/...
                             (slope_Upper_FR - slope_Lower_FR);                 % find the IC x point
    IC_FR_Y                = slope_Upper_FR * IC_FR_X + ...
                             offset_Upper_FR;                                   % find the IC x point
    
    IC_FR = [WC_FL(1), IC_FR_X, IC_FR_Y];
    
    
    %Front left IC to Front right CP
    coefficients_FL_to_CP = polyfit([IC_FL(2), DINAMIC_CP_L_Y(ii)]...          % coefficients of line through Front left IC to Front right CP
                            , [IC_FL(3), DINAMIC_CP_L_Z(ii)], 1);
    slope_FL_to_CP        = coefficients_FL_to_CP (1);                         % slope of line through Front left IC to Front Right CP
    offset_FL_to_CP       = coefficients_FL_to_CP (2);                         % offset of line through Front left IC to Front Right CP
    
    %Front right IC to Front left CP
    coefficients_FR_to_CP = polyfit([IC_FR(2), DINAMIC_CP_R_Y(ii)]...          % coefficients of line through Front right IC to Front left CP
                            , [IC_FR(3), DINAMIC_CP_R_Z(ii)], 1);
    slope_FR_to_CP        = coefficients_FR_to_CP (1);                         % slope of line through Front right IC to Front left CP
    offset_FR_to_CP       = coefficients_FR_to_CP (2);                         % offset of line through Front right IC to Front left CP
    
    
    RC_FY(ii)                 = (offset_FR_to_CP - offset_FL_to_CP)/...
                                    (slope_FL_to_CP - slope_FR_to_CP);         % find the IC x point
    RC_FZ(ii)                 = slope_FL_to_CP * RC_FY(ii) + ...
                                     offset_FL_to_CP;                          % find the IC x point
    
    RC_Front(ii, 1:3)         = [WC_FL(1), RC_FY(ii), RC_FZ(ii)];
    
    % Roll Angle
    
    phi(ii)            =  atan((DINAMIC_CP_R_Z(ii) - DINAMIC_CP_L_Z(ii))/...
                          (DINAMIC_CP_L_Y(ii) - DINAMIC_CP_R_Y(ii)))*180/pi;
end

%% Roll Center, Camber and Toe Plot

     t = output.tout; 
     p1 = polyfit(t,phi(1:length(t)), 5);
     phi = polyval(p1,t);
     
    % Camber
     
     pCamber           = polyfit(t,Camber_angle(1:length(t)),1);
     Camber            = polyval(pCamber,t);
     Camber_derivative = diff(Camber)./diff(phi);
     CamberDesired     = interp1(phi(1:end-1), Camber_derivative, 0);

     figure('NumberTitle', 'off', 'Name', 'Camber gain on phi');
     plot(phi(1:end-1), Camber_derivative);
     hold on
     plot(0, CamberDesired, 'r*');
     xlabel('phi');
     ylabel('Camber derivative');
     legend('Camber gain','Camber in phi(0)', 'Location', 'northwest');

     % Toe

     pToe           = polyfit(t,Toe_angle(1:length(t)),5);
     Toe            = polyval(pToe,t);
     Toe_derivative = diff(Toe)./diff(phi);
     ToeDesired     = interp1(phi(1:end-1), Toe_derivative, 0);

     figure('NumberTitle', 'off', 'Name', 'Toe gain on phi');
     plot(phi(1:end-1), Toe_derivative);
     hold on
     plot(0, ToeDesired, 'r*');
     xlabel('phi');
     ylabel('Toe derivative');
     legend('Toe gain','Toe in phi(0)', 'Location', 'northwest');
      
     % RC_FZ
     
     pz              = polyfit(t,RC_FZ(1:length(t)),5);
     RC_z            = polyval(pz,t);
     RC_z_derivative = diff(RC_z)./diff(phi);
     zDesired = interp1(phi(1:end-1),RC_z_derivative, 0);
     
     
     
     figure('NumberTitle', 'off', 'Name', 'Roll center z gain')
     plot(phi(1:end-1), RC_z_derivative);
     ylim([-20,20]);
     hold on
     plot(phi, RC_z);
     hold on
     plot(0, zDesired, 'r*');
     xlabel('Roll angle');
     ylabel('Roll center z axis derivative');
     legend('Roll center z gain','Roll center z in phi(0)', 'Location', 'northwest');
     
     % RC_FY
     
     py = polyfit(t,RC_FY(1:length(t)),5);
     RC_y  = polyval(py,t);
     RC_y_derivative = diff(RC_y)./diff(phi);
     yDesired = interp1(phi(1:end-1),RC_y_derivative, 0);
     
     figure('NumberTitle', 'off', 'Name', 'Roll center y gain')
     plot(phi(1:end-1), RC_y_derivative);
     hold on
     plot(phi, RC_y);
     hold on
     plot(0, yDesired, 'r*');
     xlabel('Roll angle');
     ylabel('Roll center y axis derivative');
     legend('Roll center y gain','Roll center y in phi(0)', 'Location', 'northwest');
     
     % Camber on roll angle and toe
     
     figure('NumberTitle', 'off', 'Name', 'Camber on roll angle and toe');
     T       = zeros(length(Camber), 3);
     T(:, 1) = phi;
     T(:, 2) = Toe;
     T(:, 3) = Camber;
     surf(T);
     shading flat
     hold on
     xlabel('Roll angle');
     ylabel('Toe');
     zlabel('Camber');
      
      