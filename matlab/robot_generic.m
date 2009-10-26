function robot = robot_generic( isFixed, nb_bd)

len = [0.1  0.5 0.4 0.2];
mass   = [1000000000 1.0 0.8 0.2];
M=diag([1 1 1 2 3 4]);

index = 1;

if isFixed
    arm.bd(index).name = 'ground';
    arm.bd(index).shape(1).type = 'parallelepiped';
    arm.bd(index).shape(1).dims = [len(1)  len(1)  len(1)];
    arm.bd(index).E = [0 0 0 0 0 0]';
    arm.bd(index).H_0_0L1 = eye(4);
    arm.bd(index).shape(1).H = eye(4);
    arm.bd(index).H_1_1L0 = eye(4);
    arm.bd(index).M = transport_mass_matrix( mass_parallelepiped(mass(1), [len(1) len(1) len(1)]), arm.bd(1).shape(1).H);
    arm.bd(index).shape(1).gr_props = {'FaceColor','Visible'; [1 0 0] ,'on'};

    for j=2:5
        arm.bd(index).shape(j).type = 'sphere';
        arm.bd(index).shape(j).dims = len(1)/2;
        arm.bd(index).shape(j).gr_props = {'FaceColor','Visible'; [1 1 0] ,'on'};
        arm.bd(index).shape(j).H =eye(4);
        arm.bd(index).shape(j).contact = 1;
        arm.bd(index).shape(j).material = 'skin';
    end
    arm.bd(index).shape(2).H = arb_rotvec2htr(eye(3), [len(1)/2 len(1)/2 0]);
    arm.bd(index).shape(3).H = arb_rotvec2htr(eye(3), [len(1)/2 -len(1)/2 0]);
    arm.bd(index).shape(4).H = arb_rotvec2htr(eye(3), [-len(1)/2 -len(1)/2 0]);
    arm.bd(index).shape(5).H = arb_rotvec2htr(eye(3), [-len(1)/2 len(1)/2 0]);
    
    index = index+1;
    
    arm.bd(index).E = [0 0 0 1 0 0]';
else
    arm.bd(index).E = [0 0 0 0 0 0]';
end
arm.bd(index).name = 'forearm';
arm.bd(index).shape(1).type = 'parallelepiped';
arm.bd(index).shape(1).dims = [len(2)/10  len(2)/10  len(2)];
arm.bd(index).H_0_0L1 = eye(4);
arm.bd(index).shape(1).H = arb_rotvec2htr(eye(3), [0 0 len(2)/2]);
arm.bd(index).H_1_1L0 = eye(4);
arm.bd(index).M = transport_mass_matrix(M, arm.bd(index).shape(1).H);
arm.bd(index).shape(1).gr_props = {'FaceColor','Visible'; [1 0 0] ,'on'};
index = index+1;

if nb_bd>=2
    arm.bd(index).name = 'arm';
    arm.bd(index).shape(1).type = 'parallelepiped';
    arm.bd(index).shape(1).dims = [len(3)/10  len(3)/10  len(3)];
    arm.bd(index).E = [0 0 0 1 0 0]';
    arm.bd(index).H_0_0L1 = arb_rotvec2htr(eye(3), [0 0 len(2)]);
    arm.bd(index).shape(1).H = arb_rotvec2htr(eye(3), [0 0 len(3)/2]);
    arm.bd(index).H_1_1L0 = eye(4);
    arm.bd(index).M = transport_mass_matrix(M, arm.bd(index).shape(1).H);
    arm.bd(index).shape(1).gr_props = {'FaceColor','Visible'; [1 0 0] ,'on'};
    index = index+1;
end

if nb_bd>=3
    arm.bd(index).name = 'hand';
    arm.bd(index).shape(1).type = 'parallelepiped';
    arm.bd(index).shape(1).dims = [len(4)/10  len(4)/10  len(4)];
    arm.bd(index).E = [0 0 0 1 0 0]';
    arm.bd(index).H_0_0L1 = arb_rotvec2htr(eye(3), [0 0 len(3)]);
    arm.bd(index).shape(1).H = arb_rotvec2htr(eye(3), [0 0 len(4)/2]);
    arm.bd(index).H_1_1L0 = eye(4);
    arm.bd(index).M = transport_mass_matrix(M, arm.bd(index).shape(1).H);
    arm.bd(index).shape(1).gr_props = {'FaceColor','Visible'; [1 0 0] ,'on'};
end

tree.name  = 'generic_arm';
tree.br(1) = arm;
tree.br(1).root_jk = {0 0};
icub_object = arb_treestructtree(tree);
robot = arb_robot(icub_object);
    
end % end of function
