%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Samle time 0.4s
% Prediction horizon 30 steps
% State vector dim(x)=6
% control dim(u) = 4
%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
Ts = 0.4;
p = 30;
nx = 6;
nu = 4;
nlobj = nlmpcMultistage(p,nx,nu);
nlobj.Ts = Ts;

%%
% Specify predictionn model
%%
nlobj.Model.StateFcn = "FlyingRobotStateFcn";
nlobj.Model.StateJacFcn = @FlyingRobotStateJacobianFcn;

% Specify cost function at each stage
for ct = 1:p
    nlobj.Stages(ct).CostFcn = 'FlyingRobotCostFcn';
end

% The goal is to park the robot at [0,0]
nlobj.Model.TerminalState = zeros(6,1);

% Manipulated Variables (control input) has operating range between 0 and 1
for ct = 1:nu
    nlobj.MV(ct).Min = 0;
    nlobj.MV(ct).Max = 1;
end

% Specify initial conditions
x0 = [-10;-10;pi/2;0;0;0];  % robot parks at [-10, -10], facing north
u0 = zeros(nu,1);           % thrust is zero

%% Run this Cell to plot optimal trajectory
[~,~,info] = nlmpcmove(nlobj,x0,u0);
FlyingRobotPlotPlanning(info,Ts);

%% Path following

ny = 6;
nlobj_tracking = nlmpc(nx,ny,nu);

nlobj_tracking.Model.StateFcn = nlobj.Model.StateFcn;
nlobj_tracking.Jacobian.StateFcn = nlobj.Model.StateJacFcn;

nlobj_tracking.Ts = Ts;
nlobj_tracking.PredictionHorizon = 10;
nlobj_tracking.ControlHorizon = 4;

%For tracking, tracking error has higher priority (larger penalty) than
%control efforts (smaller penalty)
nlobj_tracking.Weights.ManipulatedVariablesRate = 0.2*ones(1,nu);
nlobj_tracking.Weights.OutputVariables = 5*ones(1,nx);

% Same bounds for the thruster inputs
for ct = 1:nu
    nlobj_tracking.MV(ct).Min = 0;
    nlobj_tracking.MV(ct).Max = 1;
end

% u(1) * u(2)=0 for all prediction steps.Similar for u(3),u(4).
nlobj_tracking.Optimization.CustomEqConFcn = ...
    @(X,U,data) [U(1:end-1,1).*U(1:end-1,2); U(1:end-1,3).*U(1:end-1,4)];

validateFcns(nlobj_tracking,x0,u0);

%% Nonlinear State Estimation
% EKF is used fo
% r nonlinear state estimation.

DStateFcn = @(xk,uk,Ts) FlyingRobotStateFcnDiscreteTime(xk,uk,Ts);
DMeasFcn = @(xk) xk(1:3);
EKF = extendedKalmanFilter(DStateFcn,DMeasFcn,x0);
EKF.MeasurementNoise = 0.01;
%% Closed loop simulation of tracking control
Tsteps = 32;
xHistory = x0';
uHistory = [];
lastMV = zeros(nu,1);

Xopt = info.Xopt;
Xref = [Xopt(2:p+1,:);repmat(Xopt(end,:),Tsteps-p,1)];

hbar = waitbar(0,'Simulation Progress');
options = nlmpcmoveopt;

for k = 1:Tsteps

    % Obtain plant output measurements with sensor noise.
    yk = xHistory(k,1:3)' + randn*0.01;

    % Correct state estimation based on the measurements.
    xk = correct(EKF, yk);

    % Compute the control moves with reference previewing.
    [uk,options] = nlmpcmove(nlobj_tracking,xk,lastMV,Xref(k:min(k+9,Tsteps),:),[],options);

    % Predict the state for the next step.
    predict(EKF,uk,Ts);

    % Store the control move and update the last MV for the next step.
    uHistory(k,:) = uk'; %#ok<*SAGROW>
    lastMV = uk;

    % Update the real plant states for the next step by solving the
    % continuous-time ODEs based on current states xk and input uk.
    ODEFUN = @(t,xk) FlyingRobotStateFcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');

    % Store the state values.
    xHistory(k+1,:) = YOUT(end,:);

    % Update the status bar.
    waitbar(k/Tsteps, hbar);
end
close(hbar)

FlyingRobotPlotTracking(info,Ts,p,Tsteps,xHistory,uHistory);