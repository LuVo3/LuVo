 %% Script to calculate the Camber and toe gain of the car getting data from simscape
...multibody
    
clear
clc
close all

data

addpath('Toolbox')
addpath('CAD files/Step')
set(0, 'DefaultFigureWindowStyle', 'docked')

%% Import data from simscape multibody

output = sim('Front_Axle_Jounce','ReturnWorkspaceOutputs','on');

Camber_angle  = (output.Camber.data)*180/pi;
Toe_angle     = (output.Toe.data)*180/pi;
Jounce_travel = output.Jounce.data;

%% Angles Plot

      t = output.tout; 
      p1 = polyfit(t,Jounce_travel(1:length(t)), 5);
      Jounce = polyval(p1,t);
      
     % Camber
     
     pCamber           = polyfit(t,Camber_angle(1:length(t)),5);
     Camber            = polyval(pCamber,t);
     Camber_derivative = diff(Camber)./diff(Jounce);
     CamberDesired     = interp1(Jounce(1:end-1), Camber_derivative, 0);
     
     figure('NumberTitle', 'off', 'Name', 'Camber gain on jounce');
     plot(Jounce(1:end-1), Camber_derivative);
     hold on
     plot(Jounce, Camber);
     hold on
     plot(0, CamberDesired, 'r*');
     xlabel('Jounce');
     ylabel('Camber derivative');
     legend('Camber gain', 'Camber', 'Camber in Jounce(0)', 'Location', 'northwest');
     
     % Toe
     
     pToe           = polyfit(t,Toe_angle(1:length(t)),5);
     Toe            = polyval(pToe,t);
     Toe_derivative = diff(Toe)./diff(Jounce);
     ToeDesired     = interp1(Jounce(1:end-1), Toe_derivative, 0);
     
     figure('NumberTitle', 'off', 'Name', 'Toe gain on jounce');
     plot(Jounce(1:end-1), Toe_derivative);
     hold on
     plot(Jounce, Toe);
     hold on
     plot(0, ToeDesired, 'r*');
     xlabel('Jounce');
     ylabel('Toe derivative');
     legend('Toe gain', 'Toe', 'Toe in Jounce(0)', 'Location', 'northwest');