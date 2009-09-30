function [control self  constraintCell model] = ...
    control(self,robot,iRobot,robotArray,t,dt,constraintCell)
%          
% [controlTorque controlWrench self] = control(self,robot,iRobot,simu)
%
% The control() method compute generalized torques and external wrenches
% (in body frame) that will be applied to the robot by the integrator.
%
% If controlTorque and/or externalWrench are empty, they will not be
% taken into account. Otherwise, controlTorque is a (nDof)x1 vector
% and controlWrench is a (6*nBody)x1 vector.
%
% The inputs are :
%  robot: the controlled robot dynamical model and state
%  iRobot: the robot index in the robots array
%  robotArray: the robots array
%  t: the current time
%  dt: the time step value
%  constraintCell: the state of the constraints (contacts...)


control(1).torque = zeros(ndof(robot.tree),1);
control(1).wrench = zeros(6*nbody(robot.tree),1);


%% pour récupérer position et vitesse articulaires du robot
model = robot.model;
q  = model.q;
dq = model.T;
M  = model.inertia;
N  = model.nleffect;

disp(t);

self.N = N;
self.M = M;

tau = zeros(size(q));
%% Application des couples
control(1).torque(7:size(dq,1))=tau;