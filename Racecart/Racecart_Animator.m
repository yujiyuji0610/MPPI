%% ============================================================ %%
%%  Racecart Animator
%% ============================================================ %%

function [] = Racecart_Animator(state_log)

x     = state_log(1,:);
theta = state_log(2,:);

pole_length      = 0.8;
pole_width       = 0.05;
rectangle_width  = 1;
rectangle_height = 0.2;
cart_radius      = 0.1; 

for i = 1:length(x)   
    if rem(i,2) == 0
    
        rectangle_xcoordinate = [x(i)-rectangle_width/2;
                                 x(i)+rectangle_width/2;
                                 x(i)+rectangle_width/2;
                                 x(i)-rectangle_width/2];

        rectangle_ycoordinate = [-rectangle_height/2;
                                 -rectangle_height/2;
                                 rectangle_height/2;
                                 rectangle_height/2];

        fill(rectangle_xcoordinate,rectangle_ycoordinate,'r');
        hold on;
        axis equal;
        
        t = 0:360;
        reerwheel_xcoordinate = x(i)-0.8*rectangle_width/2+cart_radius*cos(t*pi/180);
        reerwheel_ycoordinate = rectangle_ycoordinate(1)-cart_radius*(1+sin(t*pi/180));
        frontwheel_xcoordinate = x(i)+0.8*rectangle_width/2+cart_radius*cos(t*pi/180);
        frontwheel_ycoordinate = rectangle_ycoordinate(1)-cart_radius*(1+sin(t*pi/180));
        
        fill(reerwheel_xcoordinate,reerwheel_ycoordinate,[160/255 160/255 160/255]);
        hold on;
        fill(frontwheel_xcoordinate,frontwheel_ycoordinate,[160/255 160/255 160/255]);
        hold on;
        
        t = 0:180;
        pole_xcoordinate = x(i)+pole_length*sin(theta(i))-pole_width*cos(theta(i)+t*pi/180);
        pole_ycoordinate = -pole_length*cos(theta(i))-pole_width*sin(theta(i)+t*pi/180);
        
        t = 0:180;
        pole_xcoordinate = [pole_xcoordinate x(i)+pole_width*cos(theta(i)+t*pi/180)];
        pole_ycoordinate = [pole_ycoordinate pole_width*sin(theta(i)+t*pi/180)];
        
        fill(pole_xcoordinate,pole_ycoordinate,'y');
        hold on;
        
        xlim([min(x)-2 max(x)+2]);
        ylim([-1 1]);
        hold off;
        drawnow;
    
    end                       
end
           
end