%% ============================================================ %%
%%  CartPole Simulation
%% ============================================================ %%

clear all;

% CartPole state
state = zeros(5,1);
pre_u_seq = [0];

N = 1500;
state_log = zeros(5,N);
control_log = zeros(1,N);

for i = 1:N
    
    state_log(:,i) = state;
    
    %
    if rem(i-1,2) == 0
        if i == 1
            control = CartPole_MPPI(state);
        else
            control = CartPole_MPPI(state,control);
        end
    end
    %}
    
    state = CartPole_Dynamics(state,control(:,1),0.01);
    control_log(:,i) = control(:,1);
end

%% ============================================================ %%
%% NewFile and plotdata(png) creating (Taking account of the date and time)            
%% ============================================================ %%
%{
% Newfile Name
FileName = strrep(strrep(strcat('CartPolematFile_',datestr(datetime('now'))),':','_'),' ','_');

% present date and time
date = strrep(strrep(strcat('_',datestr(datetime('now')),'.png'),':','_'),' ','_');
mat = strrep(strrep(strcat('_',datestr(datetime('now')),'.mat'),':','_'),' ','_');

% creating Newfile
mkdir(FileName);

% creating pngfile from plotfigure
save([FileName,'/result',mat]);
%}