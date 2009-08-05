clear all; 
close all;

r=0;
test=0;
a= 15;
b=5;
b_go=0.5; %b_go
u_go=4; %u_go
omega=2*pi;

x_d(1)=0.0;
y_d(1)=0.0;

x_r(1)=0.0;
y_r(1)=0.01;
xx_r(1)=0.0;

x_d2(1)=0.0;
y_d2(1)=0.0;

x_r2(1)=0.0;
y_r2(1)=0.0;

x_d3(1)=0.0;
y_d3(1)=0.0;

x_r3(1)=0.0;
y_r3(1)=0.01;

x_d4(1)=0.0;
y_d4(1)=0.0;

x_r4(1)=0.0;
y_r4(1)=0.0;


x_c(1)=0.0;
y_c(1)=1.0;

x(1)=0;
dt=0.001;
T(1)=0;
l(1)=0.0;

alpha_x=100;
alpha_y=100;

theta_left=0.0;
theta_right=0.0;
epsilon_c =4.0;
m_clock = 1.0;
theta_i=3.14;
epsilon_i=2.0;

nu=0.5;
omega=2*pi*nu;

m=1;
m2=1;
m3=1;
m4=1;
g=0;
g2=0;
g3=0;
g4=0;

beat_left=0;
beat_right=0;
previous_beat_left=0;
previous_beat_right=0;

diff=theta_left/omega;

for i = 1:(6.0/dt)

    if(T(i)*nu > beat_right)
        beat_right = previous_beat_right+1;
    end
    
    diff=theta_left/omega;
   
    if((T(i)+diff)*nu > beat_left)
        beat_left = previous_beat_left+1;
    end
     
      if(beat_right>previous_beat_right) 
         %g=g+0.1;
         previous_beat_right=beat_right;
      end
      
      if(beat_left>previous_beat_left)
         %g2=g2+0.1;
         previous_beat_left=beat_left;
      end
      
          
   r_clock=x_c(i)^2+y_c(i)^2;
   dx_c(i) = a * (m_clock-r_clock) * x_c(i) - omega* y_c(i);
   dy_c(i) = a * (m_clock-r_clock) * y_c(i) + omega*x_c(i); 
     
   dl(i)=b_go*(-l(i)+u_go);
   
   %shoulders left and right 
   
   dy_d(i) = u_go^4*b*(b/4*(g-x_d(i))- y_d(i));
   dx_d(i) = l(i)^4*y_d(i);
    
   dy_d2(i) = u_go^4*b*(b/4*(g2-x_d2(i))- y_d2(i));
   dx_d2(i) = l(i)^4*y_d2(i);
     
   r = (x_r(i)-x_d(i))^2 + y_r(i)^2;
   dx_r(i)= a*(m-r)*(x_r(i)-x_d(i))-omega*y_r(i)+epsilon_c*(cos(theta_right)*x_c(i)-sin(theta_right)*y_c(i));
   dy_r(i)= a*(m-r)*y_r(i) + omega*(x_r(i)-x_d(i))+epsilon_c*(cos(theta_right)*y_c(i)+sin(theta_right)*x_c(i));
   
   r2 = (x_r2(i)-x_d2(i))^2 + y_r2(i)^2;
   dx_r2(i)= a*(m2-r2)*(x_r2(i)-x_d2(i))-omega*y_r2(i)+epsilon_c*(cos(theta_left)*x_c(i)-sin(theta_left)*y_c(i));
   dy_r2(i)= a*(m2-r2)*y_r2(i) + omega*(x_r2(i)-x_d2(i))+epsilon_c*(cos(theta_left)*y_c(i)+sin(theta_left)*x_c(i));
  
   xx_r(i)=x_r(i);
   if(xx_r(i)<-0.5)
       if(beat_left>1)
           if(beat_left<4)
               xx_r(i)=-0.5;
           end
       end
   end
    dx_r(i)= dx_r(i)+alpha_x*(xx_r(i)-x_r(i));
    dy_r(i)= dy_r(i);%/(1+alpha_y*(xx_r(i)-x_r(i))^2);
    
   
   %elbows left and right
    
   dy_d3(i) = u_go^4*b*(b/4*(g3-x_d3(i))- y_d3(i));
   dx_d3(i) = l(i)^4*y_d3(i);
    
   dy_d4(i) = u_go^4*b*(b/4*(g4-x_d4(i))- y_d4(i));
   dx_d4(i) = l(i)^4*y_d4(i);
   
   r3 = (x_r3(i)-x_d3(i))^2 + y_r3(i)^2;
   dx_r3(i)= a*(m3-r3)*(x_r3(i)-x_d3(i))-omega*y_r3(i)+epsilon_c*(cos(theta_right-3.14)*x_c(i)-sin(theta_right-3.14)*y_c(i));
   dy_r3(i)= a*(m3-r3)*y_r3(i) + omega*(x_r3(i)-x_d3(i))+epsilon_c*(cos(theta_right-3.14)*y_c(i)+sin(theta_right-3.14)*x_c(i));
   
   r4 = (x_r4(i)-x_d4(i))^2 + y_r4(i)^2;
   dx_r4(i)= a*(m4-r4)*(x_r4(i)-x_d4(i))-omega*y_r4(i)+epsilon_c*(cos(theta_left-3.14)*x_c(i)-sin(theta_left-3.14)*y_c(i));
   dy_r4(i)= a*(m4-r4)*y_r4(i) + omega*(x_r4(i)-x_d4(i))+epsilon_c*(cos(theta_left-3.14)*y_c(i)+sin(theta_left-3.14)*x_c(i));
 
   % INTERNAL COUPLINGS
   
%    dx_r(i)=dx_r(i)+ epsilon_i*(cos(theta_i)*x_r3(i)-sin(theta_i)*y_r3(i));
%    dy_r(i)=dy_r(i)+epsilon_i*(cos(theta_i)*y_r3(i)+sin(theta_i)*x_r3(i));
%    
%    dx_r3(i)=dx_r3(i)+ epsilon_i*(cos(-theta_i)*x_r(i)-sin(-theta_i)*y_r(i));
%    dy_r3(i)=dy_r3(i)+epsilon_i*(cos(-theta_i)*y_r(i)+sin(-theta_i)*x_r(i));
%    
%    dx_r2(i)=dx_r2(i)+ epsilon_i*(cos(theta_i)*x_r4(i)-sin(theta_i)*y_r4(i));
%    dy_r2(i)=dy_r2(i)+epsilon_i*(cos(theta_i)*y_r4(i)+sin(theta_i)*x_r4(i));
%    
%    dx_r4(i)=dx_r4(i)+ epsilon_i*(cos(-theta_i)*x_r2(i)-sin(-theta_i)*y_r2(i));
%    dy_r4(i)=dy_r4(i)+epsilon_i*(cos(-theta_i)*y_r2(i)+sin(-theta_i)*x_r2(i));
   
   T(i+1) = T(i) + dt;
        
   x_d(i+1) = x_d(i) + dx_d(i)*dt;
   y_d(i+1) = y_d(i) + dy_d(i)*dt;
   x_r(i+1) = x_r(i) + dx_r(i)*dt;
   y_r(i+1) = y_r(i) + dy_r(i)*dt;
   x_d2(i+1) = x_d2(i) + dx_d2(i)*dt;
   y_d2(i+1) = y_d2(i) + dy_d2(i)*dt;
   x_r2(i+1) = x_r2(i) + dx_r2(i)*dt;
   y_r2(i+1) = y_r2(i) + dy_r2(i)*dt;
   x_d3(i+1) = x_d3(i) + dx_d3(i)*dt;
   y_d3(i+1) = y_d3(i) + dy_d3(i)*dt;
   x_r3(i+1) = x_r3(i) + dx_r3(i)*dt;
   y_r3(i+1) = y_r3(i) + dy_r3(i)*dt;
   x_d4(i+1) = x_d4(i) + dx_d4(i)*dt;
   y_d4(i+1) = y_d4(i) + dy_d4(i)*dt;
   x_r4(i+1) = x_r4(i) + dx_r4(i)*dt;
   y_r4(i+1) = y_r4(i) + dy_r4(i)*dt;
   x_c(i+1) = x_c(i) + dx_c(i)*dt;
   y_c(i+1) = y_c(i) + dy_c(i)*dt;
    
    l(i+1)=l(i)+dl(i)*dt;
          
end

dx_d(i+1)=dx_d(i);
dx_r(i+1)=dx_r(i);
%h(i+1)=h(i);
l(i+1)=l(i);

xx_r(i+1)=xx_r(i);

figure(1) 
plot(T, x_r, T, x_r2,'.', T, xx_r,'--','LineWidth', 4)









    
