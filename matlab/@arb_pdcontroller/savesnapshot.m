function snap = savesnapshot(self)
% snap = save_snapshot(self)
%  
% save_snapshot is called during the simulation to save the controller
% state (and then load in back to play the simulation)
%
% Actually, this very function saves nothing. If you want to save and load
% your controller's state, overload both the save_snapshot() and
% load_snapshot() methods

snap=struct('N', self.N, 'M', self.M);
