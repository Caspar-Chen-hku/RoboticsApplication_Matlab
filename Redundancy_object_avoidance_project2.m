clear all
close all
% Link length
l1 = 2;
l2 = 2;
l3 = 2;


%Initial joint angles in radian

theta1 = 1.7;
theta2 = -0.5;
theta3 = 1;

% Each Joint Positions using forward kinematic. 
    
px1 = l1*cos(theta1);
py1 = l1*sin(theta1);

px2 = l1*cos(theta1)+l2*cos(theta2+theta1);
py2 = l1*sin(theta1)+l2*sin(theta2+theta1);

px3 = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);
py3 = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);
       
% Target Point-calcuated by forward kinematic using the pre-defined joint angles....    

tp = [px3; py3]'; % Target position 

% centre of the circle obstacle

cen_ox = 0.1; % centre of the circle obstacle x position
cen_oy = 2; % centre of the circle obstacle y position

ox = cen_ox;
oy = cen_oy;
  
% Points values surround the circle obstacle to draw circle.(leave this unchanged)
    ang=0:0.01:2*pi; 
    
    oxp=0.4*cos(ang)+cen_ox;
    oyp=0.4*sin(ang)+cen_oy;

% a constant vlaue for increment of q0
    k0 = 0.1;
    
        
% Identity Matrix
    
In = [1 0 0; 0 1 0; 0 0 1];

steps=50;

%%
%your project start from here

for i = 1:steps
%Set velocity to zero, only allow internal motion
ve = [0;0;];


% complete the inverse kinematics using Jacobian for the redundant
% manipulator
J=[-py3 -(py3-py1) -(py3-py2);
   px3 px3-px1 px3-px2];

Jr = J'*inv(J*J');


% the distance  between P1,P2,P3 and the centre of the obstacle (cen_ox,
% cen_oy)

distance = [sqrt((px1-ox)*(px1-ox)+(py1-oy)*(py1-oy)); 
          sqrt((px2-ox)*(px2-ox)+(py2-oy)*(py2-oy));
           sqrt((px3-ox)*(px3-ox)+(py3-oy)*(py3-oy))];

%distance = [(2*l1*sin(theta1)*(ox - l1*cos(theta1)) - 2*l1*cos(theta1)*(oy - l1*sin(theta1)))/(2*((ox - l1*cos(theta1))^2 + (oy - l1*sin(theta1))^2)^(1/2)); 0; 0];
 
%"distance" should be a 3x1 vector

% select a joint which has minimum distance from the centre of obstacle

[min_dis min_joint]= min(distance);

% use the provided differentiation.m to compute the fomulis dw/dq, and let w_pq denotes dw/dq
%w = min_dis;

w_pq = [(2*l1*sin(theta1)*(ox -l1*cos(theta1)) - 2*l1*cos(theta1)*(oy-l1*sin(theta1)))/(2*((ox-l1*cos(theta1))^2 + (oy-l1*sin(theta1))^2)^(1/2))  0  0];

%compute the q0dot 
q0dot = k0*w_pq'; 

%compute qdot

qdot = (In - Jr*J)*q0dot;

theta1=qdot(1)+theta1;
theta2=qdot(2)+theta2;
theta3=qdot(3)+theta3;

px1 = l1*cos(theta1);
py1 = l1*sin(theta1);

px2 = l1*cos(theta1)+l2*cos(theta2+theta1);
py2 = l1*sin(theta1)+l2*sin(theta2+theta1);

px3 = l1*cos(theta1)+l2*cos(theta2+theta1)+l3*cos(theta3+theta2+theta1);
py3 = l1*sin(theta1)+l2*sin(theta2+theta1)+l3*sin(theta3+theta2+theta1);


% distances of P1,2,3 to the object
dis_j1(i) = distance(1);
dis_j2(i) = distance(2);
dis_j3(i) = distance(3);

% position error of the endeffector
error_p(i) = sum([abs(tp(1)-px3) abs(tp(2)-py3)]);


figure(1),
plot(tp(1), tp(2),'r*')
plot(px3, py3,'k*')
plot(oxp,oyp,'-k','LineWidth',4)
line([0 px1],[0 py1],'Color','r','LineWidth',4)
line([px1 px2],[py1 py2],'Color','g','LineWidth',4)
line([px2 px3],[py2 py3],'Color','b','LineWidth',4)
axis([-3 3,0,6])
drawnow;
pause(0.1)


end



%plotting, you can skip this part
plot_size = steps;
    
    figure(2), 
    hold on
    plot(1:1:plot_size,dis_j1,'r','LineWidth',4)
    plot(1:1:plot_size,dis_j2,'b','LineWidth',4)
    plot(1:1:plot_size,dis_j3,'k','LineWidth',4)
    set(gca,'FontSize',10,'fontWeight','bold')
    grid on
    xlabel('steps','FontSize', 15)
    ylabel('The distace from obstacle','FontSize', 15)

    figure(3), plot(1:1:plot_size,error_p,'r','LineWidth',4)
    hold on
    set(gca,'FontSize',10,'fontWeight','bold')
    grid on
    xlabel('steps','FontSize', 15)
    ylabel('The position error','FontSize', 15)
    
    
 