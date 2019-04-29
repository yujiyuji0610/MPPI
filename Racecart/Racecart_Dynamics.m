%% ============================================================ %%
%%  Racecart Dynamics
%% ============================================================ %%

function next_state = Racecart_Dynamics(state,control,dt)

g     = 9.81;
M     = 1724;
Iz    = 1300;
a     = 1.35;
b     = 1.15;
CF    = 1.2*10^5;
CR    = 1.75*10^5;
mu    = 0.55;

x     = state(1);
y     = state(2);
psi   = state(3); 
beta  = state(4);
vx    = state(5);
vy    = state(6);
r     = state(7);
delta = state(8);
Fx    = state(9);

Fz     = M*g;
aF     = atan(beta+a*r/vx)-delta;
aR     = atan(beta-b*r/vx);
xi     = sqrt(mu^2*Fz^2-Fx^2)/(mu*Fz);

gammaF = atan(3*xi*mu*Fz/CF);
gammaR = atan(3*xi*mu*Fz/CR);

if control(1) >= 90*pi/180
    %control(1) = 90*pi/180;
end
if control(2) >= 8600
    control(2) = 8600;
end

if aF == 0
    FyF = 0;
else
    if abs(aF) >= gammaF
        FyF = -mu*xi*Fz*abs(aF)/aF;
    else
        FyF = -CF*tan(aF)+CF^2/(3*xi*mu*Fz)*(tan(aF))^3/abs(tan(aF))-CF^3/(27*mu^2*xi^2*Fz^2)*(tan(aF))^3;
    end
end

if aR == 0
    FyR = 0;
else
    if abs(aR) >= gammaR
        FyR = -mu*xi*Fz*abs(aR)/aR;
    else
        FyR = -CR*tan(aR)+CR^2/(3*xi*mu*Fz)*(tan(aR))^3/abs(tan(aR))-CR^3/(27*mu^2*xi^2*Fz^2)*(tan(aR))^3;
    end
end

next_state = [x     + (vx*cos(psi)-vy*sin(psi))*dt;
             y     + (vx*sin(psi)+vy*cos(psi))*dt;
             psi   + r*dt;
             beta  + ((FyF+FyR)/M/vx-r)*dt;
             vx    + ((Fx-FyF*sin(delta))/M+r*vx*beta)*dt;
             vy    + ((FyF+FyR)/M-r*vx)*dt;
             r     + (a*FyF-b*FyR)/Iz*dt;
             delta + 10*(control(1)-delta)*dt;
             Fx    + 10*(control(2)-Fx)*dt];
                  
end