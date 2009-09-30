function val = arb_pdcontroller(robot)
%
% arb_pdcontroller() ajoute le champ état articulaire désiré au robot ainsi
% que les gains du correcteur

nDof = ndof(robot.tree);


val.N = zeros(nDof);
val.M = zeros(nDof);

val = class( val, 'arb_pdcontroller',arb_dynamiccontroller());


