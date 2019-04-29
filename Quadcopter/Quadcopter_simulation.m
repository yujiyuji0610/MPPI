%% ============================================================ %%
%%  Quadcopter Simulation
%% ============================================================ %%

clear all;

% Quadcopter state
state       = zeros(16,1);
state_diff  = zeros(16,1);
state(3)    = 10;
state(7)    = 0*pi/180;
state(8)    = 0*pi/180;
state(13:16)= 620.6108;

N = 1;
state_log = zeros(16,N);
state_diff_log = zeros(16,N);
control_log = zeros(4,N);
%T_and_Tau_log = zers(4,N);

for i = 1:N
    
    state_log(:,i)  = state;
    state_diff_log(:,i)  = state_diff;
    %
    if rem(i-1,2) == 0
        if i == 1
            control = Quadcopter_MPPI(state);
        else
            control = Quadcopter_MPPI(state,control);
        end
    end
    %}
    %{
    if rem(i-1,2) == 0
        [control, T_and_Tau] = Quadcopter_PID_controller(state,state_diff,control);
    end
    %}
    state = Quadcopter_Dynamics(state,control(:,1),0.01);
    state_diff = (state-state_log(:,i))/0.01;
    control_log(:,i) = control(:,1);
    %T_and_Tau_log(:,i) = T_and_Tau;
    
end

%% ============================================================ %%
%% NewFile and plotdata(png) creating (Taking account of the date and time)            
%% ============================================================ %%
%{
% Newfile Name
FileName = strrep(strrep(strcat('QuadcoptermatFile_',datestr(datetime('now'))),':','_'),' ','_');

% present date and time
date = strrep(strrep(strcat('_',datestr(datetime('now')),'.png'),':','_'),' ','_');
mat = strrep(strrep(strcat('_',datestr(datetime('now')),'.mat'),':','_'),' ','_');

% creating Newfile
mkdir(FileName);

% creating pngfile from plotfigure
save([FileName,'/result',mat]);
%}