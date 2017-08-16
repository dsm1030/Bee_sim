function [r,xtrag,utraj,prog] = runDircolPillarb

% simple demo of bee flying through a periodic lattice

r = Quadrotorb();
x = -12:4:8;
y1 = 8:-4:-4;
y2 = 6:-4:-6;
for i = 1:length(x)
    if rem(x(i)/2,4) == 0
        y = y2;
    else
        y = y1;
    end
    for j = 1:length(y)
        r = addPillar(r, 0.1, [x(i);y(j)]/2);
    end
end

N = 50;
minimum_duration = 0;
maximum_duration = 6;
prog = DircolTrajectoryOptimization(r,N,...
    [minimum_duration maximum_duration]);

x0 = Point(getStateFrame(r));
x0.base_x = -9;
x0.base_z = 0.5;
x0.base_xdot = 5;
x0.base_pitch = -pi/6;
u0 = double(nominalThrust(r));

if (nargout<1)
    v = constructVisualizer(r,struct('use_collision_geometry',true));
    v.draw(0,double(x0));

    prog = addPlanVisualizer(r,prog);
end
    
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

xf = x0;
xf.base_x = 9;
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
prog = prog.addInputConstraint(ConstantConstraint(u0),N);

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@final_cost);

collision_constraint = generateConstraint(MinDistanceConstraint(r,0.1),0);
prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(r));

tf0 = 6;                      % initial guess at duration

traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

prog = prog.setSolverOptions('snopt','majoroptimalitytolerance',1e-2);
prog = prog.setSolverOptions('snopt','majorfeasibilitytolerance',1e-2);

tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
toc
if (nargout<1)
  v.playback(xtraj,struct('slider',true));
end

end

function [g,dg] = cost(dt,x,u)

R = eye(4);
g = u'*R*u;
%g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
%dg = zeros(1, 1 + size(x,1) + size(u,1));

end

function [h,dh] = final_cost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))];

end