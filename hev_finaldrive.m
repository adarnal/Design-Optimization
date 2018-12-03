% HEV Vehicle Architecture Exploration Research
% Optimal Design Laboratory
% University of Michigan
% ***********************************************
% Final Drive Model File v1.0
% Created on 11/09/2012 by Alparslan Emrah Bayrak
% ***********************************************
% Parameter Definitions:
% Wveh: Vehicle shaft speed [rpm]
% Tveh: Vehicle shaft torque [Nm]
function [Ttrans, Wtrans]=hev_finaldrive(Wveh, Tveh, fd_ratio)
% Final Drive Parameters
fd_ratio = 19.217;     % Final drive ratio
fd_eff = 0.8;         % Gear efficiency

% Output Calculations  
Ttrans = Tveh/fd_ratio*fd_eff;   % Transmission torque [Nm]
Wtrans = Wveh*fd_ratio;         % Transmission speed [rpm]
end