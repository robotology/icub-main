clear all; 
close all;

c=0.05;

x_d(1)=0.0;
y_d(1)=0.0;

x_r(1)=0.0;
y_r(1)=0.01;

x_d2(1)=0.0;
y_d2(1)=0.0;

x_r2(1)=0.0;
y_r2(1)=0.0;

x_c(1)=0.0;
y_c(1)=1.0;

theta_c=0.0;
epsilon_c =8;
m_clock = 1.0;
x(1)=0;
dt=0.001;
T(1)=0;

f=0;
h(1)=0.1;
l(1)=0.0;

r=0;
test=0;
g= 0.0;
a= 15;
m=-15;
b=5;
epsilon=1;
b_go=0.5; %b_go
u_go=1; %u_go

g=-0.8552/0.1;
g2=0;

omega=2*pi*0.5;

epsilon_i =1.0;
theta_i=0.0;

m=-5;
m2=-5;
test=0;

da(1)=0;


for i = [1:(15.0/dt)]

      if(T(i)>4.3)
        if(test==0)
          l(i)=0;
          test=1       
        end
        m=-5;
        m2=-5;
        g=3;
      end
     
     if(T(i)>8.9)
           if(test==1)
                l(i)=0;
                test=2;
           end 
        m=1;
        m2=1;
        g=1;
     end
    
   r_clock=sqrt(x_c(i)^2+y_c(i)^2);
   dx_c(i) = a * (m_clock-r_clock^2) * x_c(i) - omega* y_c(i);
   dy_c(i) = a * (m_clock-r_clock^2) * y_c(i) + omega*x_c(i); 
     
   dl(i)=b_go*(-l(i)+u_go);
   
   %dy_d(i) = b*(b/4*(g-x_d(i))- y_d(i));
   %dx_d(i) = y_d(i);
   
    dy_d(i) = u_go^4*b*(b/4*(g-x_d(i))- y_d(i));
    dx_d(i) = l(i)^4*y_d(i);
%    
   dy_d2(i) = u_go^4*b*(b/4*(g2-x_d2(i))- y_d2(i));
   dx_d2(i) = l(i)^4*y_d2(i);
     
   r = (x_r(i)-x_d(i))^2 + y_r(i)^2;
   dx_r(i)= a*(m-r)*(x_r(i)-x_d(i))-omega*y_r(i)+epsilon_c*(cos(theta_c)*x_c(i)-sin(theta_c)*y_c(i));
   dy_r(i)= a*(m-r)*y_r(i) + omega*(x_r(i)-x_d(i))+epsilon_c*(cos(theta_c)*y_c(i)+sin(theta_c)*x_c(i)); 

% 
% if(i>1)
%        
%    da = dx_r(i)-dx_r(i-1)
%    if(dx_r(i)-dx_r(i-1)>max_acc)
%        dx_r(i) = dx_r(i-1)+max_acc;
%    end
%    if(dx_r(i-1)-dx_r(i)>max_acc)
%        dx_r(i)= dx_r(i-1)-max_acc;
%    end
% end
   dx_r(i)= dx_r(i)+ epsilon_i*(cos(-theta_i)*(x_r2(i)-x_d2(i))-sin(-theta_i)*y_r2(i));
   dy_r(i)= dy_r(i)+ epsilon_i*(cos(-theta_i)*y_r2(i)+sin(-theta_i)*(x_r2(i)-x_d2(i))); 
   
   r2 = (x_r2(i)-x_d2(i))^2 + y_r2(i)^2;
   dx_r2(i)= a*(m2-r)*(x_r2(i)-x_d2(i))-omega*y_r2(i)+epsilon_i*(cos(theta_i)*(x_r(i)-x_d(i))-sin(theta_i)*y_r(i));
   dy_r2(i)= a*(m2-r)*y_r2(i) + omega*(x_r2(i)-x_d2(i))+epsilon_i*(cos(theta_i)*y_r(i)+sin(theta_i)*(x_r(i)-x_d(i))); 

   
   T(i+1) = T(i) + dt;
        
   x_d(i+1) = x_d(i) + dx_d(i)*dt;
   y_d(i+1) = y_d(i) + dy_d(i)*dt;
   x_r(i+1) = x_r(i) + dx_r(i)*dt;
   y_r(i+1) = y_r(i) + dy_r(i)*dt;
   x_d2(i+1) = x_d2(i) + dx_d2(i)*dt;
   y_d2(i+1) = y_d2(i) + dy_d2(i)*dt;
   x_r2(i+1) = x_r2(i) + dx_r2(i)*dt;
   y_r2(i+1) = y_r2(i) + dy_r2(i)*dt;
   x_c(i+1) = x_c(i) + dx_c(i)*dt;
   y_c(i+1) = y_c(i) + dy_c(i)*dt;
    
    l(i+1)=l(i)+dl(i)*dt;
          
end

dx_d(i+1)=dx_d(i);
dx_r(i+1)=dx_r(i);
%h(i+1)=h(i);
l(i+1)=l(i);

figure(1) 
plot(T, x_r, T, x_d, '.', T, x_c,'--','LineWidth', 4)
legend('R+D', 'D', 'Clock')

figure(2) 
plot(T, dx_r, T,  dx_d,':', 'LineWidth', 4)
legend('R+D', 'D')







    
