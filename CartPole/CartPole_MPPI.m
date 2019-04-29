%% ============================================================ %%
%%  CartPole MPPI (Model Predictive Path Integral Control)
%% ============================================================ %%

function u_seq = CartPole_MPPI(state,previous_u_seq)

% ======================================================================= %
%   General Parameter Setting
% ======================================================================= %

K  = 500;
Hp = 50;
control_dt = 0.02;
plant_dt = 0.01;
dt = 0.02;
nu = 1000;
R  = 1;
lambda = 500;

state_dim = size(state,1);
u_dim = 1;

% ======================================================================= %
%   Generate random control variations >>> (N*K)
% ======================================================================= %

disturbance_parameter = 0.5;
epsilon_sigma = 0.6;

du_seq = disturbance_parameter*1/sqrt(dt)*normrnd(0,epsilon_sigma,[u_dim,K,Hp]);

%du_seq_1dim = normrnd(0,epsilon_sigma,[1,K,Hp]);
%du_seq = disturbance_parameter*1/sqrt(dt)*repmat(du_seq_1dim,[4 1 1]);

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
        
        %x_seq(:,i+1) = CartPole_Dynamics(x_seq(:,i),u_seq(:,i)+du_seq(:,k,i),control_dt);
        
        %
        x_plant_seq(:,1) = x_seq(:,i);
        for j = 1:(control_dt/plant_dt)
            x_plant_seq(:,j+1) = CartPole_Dynamics(x_plant_seq(:,j),u_seq(:,i)+du_seq(:,k,i),plant_dt);
        end
        x_seq(:,i+1) = x_plant_seq(:,end);
        %}
        
    end
    
    S_temp_seq = zeros(1,Hp);
    S_temp_seq(1,Hp) = CartPole_Costfunc(x_seq(:,Hp),u_seq(:,Hp),du_seq(:,k,Hp),nu,R);
    for i = 1:Hp-1
        
        S_temp_seq(1,Hp-i) = S_temp_seq(1,Hp+1-i) + CartPole_Costfunc(x_seq(:,Hp-i),u_seq(:,Hp-i),du_seq(:,k,Hp-i),nu,R);
        
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
    
    %u_seq(i,:) = u_seq(i,:) + du_seq(S_min_index,:,i);
end

end