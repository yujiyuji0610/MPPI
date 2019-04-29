%% ============================================================ %%
%%  Racecart Simulation
%% ============================================================ %%

clear all;

% Racecart state
state    = zeros(9,1);
state(5) = 0.5;

N = 3000;
state_log   = zeros(9,N);
control_log = zeros(2,N);

for i = 1:N
    
    state_log(:,i) = state;
    
    %
    if rem(i-1,2) == 0
        if i == 1
            control = Racecart_MPPI(state);
        else
            control = Racecart_MPPI(state,control);
        end
    end
    %}
    
    state = Racecart_Dynamics(state,control(:,1),0.01);
    control_log(:,i) = control(:,1);
end

%% ============================================================ %%
%% NewFile and plotdata(png) creating (Taking account of the date and time)            
%% ============================================================ %%
%{
% Newfile Name
FileName = strrep(strrep(strcat('RacecartmatFile_',datestr(datetime('now'))),':','_'),' ','_');

% present date and time
date = strrep(strrep(strcat('_',datestr(datetime('now')),'.png'),':','_'),' ','_');
mat = strrep(strrep(strcat('_',datestr(datetime('now')),'.mat'),':','_'),' ','_');

% creating Newfile
mkdir(FileName);

% creating pngfile from plotfigure
save([FileName,'/result',mat]);
%}

%% ============================================================ %%
%% Animation        
%% ============================================================ %%

%{
state_for_animation = zeros(6,N);
state_for_animation(1,:) = state_log(1,:);
state_for_animation(2,:) = state_log(2,:);
state_for_animation(6,:) = state_log(3,:);

animation_creator(state_for_animation);
%}