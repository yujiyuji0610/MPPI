%% ============================================================ %%
%%  Racecart Cost function
%% ============================================================ %%

function q = Racecart_Costfunc(state,control,du,nu,R)

x     = state(1);
y     = state(2);
psi   = state(3); 
beta  = state(4);
vx    = state(5);
vy    = state(6);
r     = state(7);
delta = state(8);
Fx    = state(9);

if abs(beta) >= 15*pi/180
    Critical_cost = 1000;
else
    Critical_cost = 0;
end

d = abs((x/13)^2+((y-13)/13)^2-1);

q = 100*d^2+(vx-7.0)^2 ...
    +(1-nu^(-1))/2*du'*R*du + control'*R*du + 1/2*control'*R*control ...
    +Critical_cost;

end