function simulate_matlab(nb_bd, T0, gravity, dt, tend, sim_name)
% Run an arboris-matlab simulation involving robot_generic and generates the equivalent arboris-python robot

    %% Initialization
    global globs; init_globs;
    
    isFixed = 0;
    %% Robot
    r = robot_generic(isFixed, nb_bd);
    r(1).T = T0(:);
    r(1).controller = arb_h5controller(r(1));
    r(1).model.GravityAcceleration = gravity(:);
    
    convert_mat2py(r, 'robot_generic.py')
    %% Simulation
    c = contact(r); c = remove_self_collision(c);
    [rs cs time] = simulate(r, c, 0.,tend,dt); % effective simulation

    %% Results
    H0 = arrayfun(@(x)( x.model.H0),rs(1,1:end),'UniformOutput',0);
    %q = arrayfun(@(x)( x.model.q),rs(1,1:end),'UniformOutput',0);
    dq = arrayfun(@(x)( x.model.T),rs(1,1:end),'UniformOutput',0);
    M = arrayfun(@(x)( x.controller.mass),rs(1,1:end),'UniformOutput',0);
    N = arrayfun(@(x)( x.controller.nleffects),rs(1,1:end),'UniformOutput',0);
    
    file_name = [sim_name '_mat.h5'];
    hdf5write(file_name, '/time', time, '/H0', H0,...
            ...'/q', q, 
            '/dq', dq, '/M', M, '/N', N);
    exit
    
end


