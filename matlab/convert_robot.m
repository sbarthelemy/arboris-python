function convert_robot(robot, filename)
% given an arboris-matlab robot (described in a tree-sctruct), generates a python 
% file that would the arboris-python equivalent.

tree = robot.tree.tree;

file = fopen(filename, 'w');
init(file);
for i=1:numel(tree.br)
    for j=1:numel(tree.br(i).bd)
        register(file, tree, i, j);
    end
end
fclose(file);

end




function init(f)
fprintf(f, '# This is an auto generated file.\n');
fprintf(f, '# This module builds a robot equivalent to an arboris-matlab one.\n\n');

fprintf(f, 'from arboris.core import Body, SubFrame\n');
fprintf(f, 'from arboris.homogeneousmatrix import transl\n');
fprintf(f, 'from arboris.joints import *\n');
fprintf(f, 'from arboris.shapes import *\n');
fprintf(f, 'from numpy import array\n');
fprintf(f, '\n\ndef add_robot(w):\n');
end

function register(f, tree, i, j)
bd=tree.br(i).bd(j);

current_body = ['br' num2str(i) 'bd' num2str(j)];

if i==1 && j==1
    parent_body = 'w.ground';
elseif j==1
    pbr=tree.br(i).root_jk{1};
    pbd=tree.br(i).root_jk{2};
    parent_body = ['br' num2str(pbr) 'bd' num2str(pbd)];
else
    parent_body = ['br' num2str(i) 'bd' num2str(j-1)];
end

fprintf(f, '    ### tree.br(%i).bd(%i) ### %s\n', i, j, bd.name)
register_body(f, bd, current_body);
register_joint(f, bd, parent_body, current_body);
register_shapes(f, bd, current_body);
fprintf(f, '\n\n');
end

function str = print_list(M)
% print an array as a python list (1D)
str = sprintf('% .8f, ', M);
str = ['[' str(1:end-2) ']'];
end


function str = print_array2(M)
[nrows, ncols] = size(M);
str = 'array([';
for i=1:nrows
    str = [str sprintf('\n        %s,', print_list(M(i,:)))];
end
str = [str(1:end-1) '])'];
end

function M = flip_mass(A)
M=zeros(6);
M(1:3,1:3) = A(4:6,4:6);
M(4:6,4:6) = A(1:3,1:3);
M(1:3,4:6) = A(4:6,1:3);
M(4:6,1:3) = A(1:3,4:6);
end

function register_body(f, bd, name)
% write body
fprintf(f, '    mass = %s\n', print_array2(flip_mass(bd.M)));
fprintf(f, '    %s = Body(name=''%s'', mass=mass)\n', name, bd.name);
end

function register_joint(f, bd, parent_body, child_body)
% write the subframes
H00L1 = bd.H_0_0L1;
if all(all(H00L1 == eye(4)))
    parent_frame = parent_body;
else
    parent_frame = 'pf';
    fprintf(f, '    pf = SubFrame(%s, %s)\n', parent_body, print_array2(H00L1));
end

H11L0 = bd.H_1_1L0;
if all(all(H11L0 == eye(4)))
    child_frame = child_body;
else
    fprintf(f, '    cf = SubFrame(%s, %s)\n', child_body, print_array2(H11L0));
    child_frame = 'cf';
end

% write joint
if isempty(bd.E)
    jtype = 'FreeJoint';
else
    if bd.E(4)==1
        jtype = 'RxJoint';
    elseif bd.E(5)==1
        jtype = 'RyJoint';
    elseif bd.E(6)==1
        jtype = 'RzJoint';
    else
        jtype = 'FreeJoint';
    end
end

fprintf(f, '    joint = %s(name=''%s'')\n', jtype, bd.name);
fprintf(f, '    w.add_link(%s, joint, %s)\n', parent_frame, child_frame);

end

function register_shapes(f, bd, body)
for k=1:numel(bd.shape)
    sh = bd.shape(k);
    if (sh.contact == 1) && ismember(sh.type, [1 2])
        if all(all(sh.H == eye(4)))
            frame = body;
        else
            % add a frame where to put the shape
            fprintf(f, '    sf = SubFrame(%s, %s)\n', body, print_array2(sh.H));
            frame = 'sf';
        end
        
        switch sh.type
            case 1
                fprintf(f, '    sh = Sphere(frame=%s, radius=%f)\n', ...
                    frame, sh.dims(1));
            case 2
                fprintf(f, '    sh = Box(frame=%s, lengths=%f)\n', ...
                    frame, print_list(sh.dims));
        end       
        fprintf(f, '    w.register(sh)\n');
    end
end
end
