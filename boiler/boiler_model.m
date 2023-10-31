%% Identified Linear Models:
% Operating point 185 MW
% TP: Throttle Pressure
% MWO: Megawatt output
% FR: Firing Rate
% TV: Turbine Valve

s = tf('s');
MWOTV = (-0.0025291*s*(s-2.12)*(s+0.1783)*(s+0.06005)*(s+0.01543))/...
    ((s+0.195)*(s+0.0752)*(s^2+0.02651*s+0.0004476)*(s^2+0.08533*s+0.002877));
MWOFR = (-0.00092731*(s-2.136)*(s^2+0.07913*s+0.002069)*(s^2-0.09745*s+0.01647))/...
    ((s+0.195)*(s+0.0752)*(s^2+0.02651*s+0.0004476)*(s^2+0.08533*s+0.002877));
TPTV = (-0.0014435*(s+0.1478)*(s^2+0.05292*s+0.001739)*(s^2-0.7298*s+0.7524))/...
    ((s+0.195)*(s+0.0752)*(s^2+0.02651*s+0.0004476)*(s^2+0.08533*s+0.002877));
TPFR = (-0.00066211*(s-5.62)*(s^2+0.08217*s+0.002909)*(s^2-0.2491*s+0.05502))/...
    ((s+0.195)*(s+0.0752)*(s^2+0.02651*s+0.0004476)*(s^2+0.08533*s+0.002877));

% Define deadzone effect parameter
% Firing rate
K2 = 3.0
j2 = 0.5
% Valve position
K1 = 1.0
j1 = 1.0

% Setpoint info
mwo_setpoint = 185
tp_setpoint = 50

% Ratio rate
N1 = 1 % radians/megawatt
N2 = 1 % radians/psi

% Actuator gains
fv1 = 0.1
fv2 = 0.1
J1 = 0.01
J2 = 0.01
