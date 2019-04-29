%% ============================================================ %%
%%  Racecart MPPI (Model Predictive Path Integral Control)
%% ============================================================ %%

function u_seq = Racecart_MPPI(state,previous_u_seq)

% ======================================================================= %
%   General Parameter Setting
% ======================================================================= %

K  = 500;
Hp = 40;
control_dt = 0.02;
plant_dt = 0.01;
dt = 0.02;
nu = 1000;
R  = [0.00001];
lambda = 50;

state_dim = size(state,1);
u_dim = 2;

% ======================================================================= %
%   Generate random control variations >>> (N*K)
% ======================================================================= %

disturbance_parameter = 0.5;
epsilon_sigma = 0.6;

ddelta_seq = normrnd(0,0.5,[K,Hp]);
ddelta_maxindex = (ddelta_seq >= 45*pi/180);
ddelta_seq(ddelta_maxindex) = 45*pi/180;
ddelta_minindex = (ddelta_seq <= -45*pi/180);
ddelta_seq(ddelta_minindex) = -45*pi/180;

dFx_seq    = normrnd(300,1000,[K,Hp]);

du_seq = zeros([u_dim,K,Hp]);
du_seq(1,:,:) = ddelta_seq;
du_seq(2,:,:) = dFx_seq;

% ======================================================================= %
%   The cost of a trajectory
% ======================================================================= %

u_init = zeros(u_dim,1);

switch nargin
    case 1 
        u_seq = 0*ones(u_dim,Hp);
    case 2
        u_seq(:,1:Hp-1) = previous_u_seq(:,2:Hp);
        u_seq(:,Hp) = previous_u_seq(1,Hp);
end

S_seq = zeros(K,Hp);

for k = 1:K
    x_seq = zeros(state_dim,Hp);
    x_plant_seq = zeros(state_dim,(control_dt/plant_dt)+1);
    x_seq(:,1) = state;
   
    for i = 1:Hp-1
        
        %x_seq(:,i+1) = Racecart_Dynamics(x_seq(:,i),u_seq(:,i)+du_seq(:,k,i),control_dt);
        
        %
        x_plant_seq(:,1) = x_seq(:,i);
        for j = 1:(control_dt/plant_dt)
            x_plant_seq(:,j+1) = Racecart_Dynamics(x_plant_seq(:,j),u_seq(:,i)+du_seq(:,k,i),plant_dt);
        end
        x_seq(:,i+1) = x_plant_seq(:,end);
        %}
        
    end
    
    S_temp_seq = zeros(1,Hp);
    S_temp_seq(1,Hp) = Racecart_Costfunc(x_seq(:,Hp),u_seq(:,Hp),du_seq(:,k,Hp),nu,R);
    for i = 1:Hp-1
        
        S_temp_seq(1,Hp-i) = S_temp_seq(1,Hp+1-i) + Racecart_Costfunc(x_seq(:,Hp-i),u_seq(:,Hp-i),du_seq(:,k,Hp-i),nu,R);
        
    end
    S_seq(k,:) = S_temp_seq;
end

du_seq = permute(du_seq,[2 3 1]);
[S_min_value, S_min_index] = min(S_seq(:,1));

S_min_seq = min(S_seq);
S_mean_seq = mean(S_seq);
S_std_seq = std(S_seq);
%S_seq = S_seq-S_min_seq+1;
for i = 1:u_dim
    u_seq(i,:) = u_seq(i,:) + (sum((exp(-(1/lambda)*S_seq)).*du_seq(:,:,i))./sum(exp(-(1/lambda)*S_seq)));
    
    if i == 1
        u_max_index = (u_seq(i,:) > 45*pi/180);
        u_seq(i,u_max_index) = 45*pi/180;
        u_min_index = (u_seq(i,:) < -45*pi/180);
        u_seq(i,u_min_index) = -45*pi/180;
    elseif i == 2
        u_max_index = (u_seq(i,:) > 8600);
        u_seq(i,u_max_index) = 8600;
    end
    
    %u_seq(i,:) = u_seq(i,:) + du_seq(S_min_index,:,i);
end

end
