clear all

a=100;
b=10;
c=0.5;

x_d(1)=2.0;
y_d(1)=0.0;

x_d2(1)=0.0;
y_d2(1)=0.0;

x_d3(1)=2.0;
y_d3(1)=0.0;

x_d4(1)=0.0;
y_d4(1)=0.0;

h(1)=0.0;
l(1)=0.0;

h2(1)=0.0;
l2(1)=0.0;

h3(1)=0.0;
l3(1)=0.0;

h4(1)=0.0;
l4(1)=0.0;

dt=0.001;
T(1)=0;

g= 3;

max=2;
gamma=1;

 
for i = [1:(10/dt)]
    
    dl(i)=gamma*(-l(i)+max);
            
    dy_d(i) = max^2*b*(b/4*(g-x_d(i))- y_d(i)); 
    dx_d(i) = l(i)^2*y_d(i);
    
    dy_d2(i) = max^2*b*(b/4*(g-x_d2(i))- y_d2(i)); 
    dx_d2(i) = l(i)^2*y_d2(i);
    
    dy_d3(i) = max^4*b*(b/4*(g-x_d3(i))- y_d3(i)); 
    dx_d3(i) = l(i)^4*y_d3(i);
    
    dy_d4(i) = max^4*b*(b/4*(g-x_d4(i))- y_d4(i)); 
    dx_d4(i) = l(i)^4*y_d4(i);
    
    T(i+1) = T(i) + dt;

    x_d(i+1) = x_d(i) + dx_d(i)*dt;
    y_d(i+1) = y_d(i) + dy_d(i)*dt;
    
    x_d2(i+1) = x_d2(i) + dx_d2(i)*dt;
    y_d2(i+1) = y_d2(i) + dy_d2(i)*dt;
    
    x_d3(i+1) = x_d3(i) + dx_d3(i)*dt;
    y_d3(i+1) = y_d3(i) + dy_d3(i)*dt;
    
    x_d4(i+1) = x_d4(i) + dx_d4(i)*dt;
    y_d4(i+1) = y_d4(i) + dy_d4(i)*dt;
    
    l(i+1)=l(i)+dl(i)*dt;
       
end

dx_d(i+1)=dx_d(i);
dx_d2(i+1)=dx_d2(i);
dx_d3(i+1)=dx_d3(i);
dx_d4(i+1)=dx_d4(i);

figure(1) 
plot(T, x_d, T, x_d2, ':',T, x_d3,'-.', T, x_d4,'--', 'LineWidth', 4)

figure(2)
plot(T, dx_d, T, dx_d2, ':',T, dx_d3,'-.', T, dx_d4,'--', 'LineWidth', 4)

