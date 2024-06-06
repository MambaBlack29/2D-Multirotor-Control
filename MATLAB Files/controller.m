function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters
%
%   Using these current and desired states, you have to compute the desired
%   controls

error.pos = des_state.pos - state.pos;
error.vel = des_state.vel - state.vel;

KZ = [100, 10]; % [KpZ, KdZ] not really affecting the motion that much :(
PIDZ = des_state.acc(2) + KZ(1)*error.pos(2) + KZ(2)*error.vel(2); % acceleration along z

u1 = params.mass*(params.gravity + PIDZ)/cos(state.rot);

KY = [18, 8]; % [KpY, KdY]
PIDY = des_state.acc(1) + KY(1)*error.pos(1) + KY(2)*error.vel(1); % acceleration along y
PhiDesired = asin(-params.mass*PIDY/u1);

KPhi = [600, 30]; % [KpPhi, KdPhi]
PIDPhi = (KPhi(1)*(PhiDesired - state.rot) - KPhi(2)*state.omega); % angular acceleration about xx

u2 = params.Ixx*PIDPhi;

end
