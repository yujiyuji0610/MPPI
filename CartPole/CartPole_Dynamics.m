%% ============================================================ %%
%%  CartPole Dynamics
%% ============================================================ %%

function next_state = CartPole_Dynamics(state,control,dt)

g     = 9.81;
mc    = 1.0;
mp    = 0.01;
l     = 0.8;

x     = state(1); 
theta = state(2); 
v     = state(3); 
omega = state(4); 
f     = state(5);

next_state = state + [v;
                      omega;
                      1/(mc+mp*sin(theta)^2)*(f+mp*sin(theta)*(l*omega^2+g*cos(theta)));
                      1/l/(mc+mp*sin(theta)^2)*(-f*cos(theta)-mp*l*omega^2*cos(theta)*sin(theta)-(mc+mp)*g*sin(theta));
                      20*(control-f)]...
                      *dt;
                  
end