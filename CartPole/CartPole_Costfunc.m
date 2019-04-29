%% ============================================================ %%
%%  CartPole Cost function
%% ============================================================ %%

function q = CartPole_Costfunc(state,control,du,nu,R)

x     = state(1); 
theta = state(2); 
v     = state(3); 
omega = state(4); 
f     = state(5);

Critical_cost = 0;

q = 80*x^2+500*(1+cos(theta))^2+10*v^2+1*omega^2 ...
    +(1-nu^(-1))/2*du'*R*du + control'*R*du + 1/2*control'*R*control ...
    +Critical_cost;

end