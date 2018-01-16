clear all

% origin of the tip of seg2
p02=[0 0]';

% unit vector along x axis in the tip local frame of seg2
p2=[1 0]';

l1_1=9;
l2_1=11;

l1_2=12;
l2_2=8;

d=1;
Lsegment = 10;



%compute the bending angle of seg1
theta1=(l2_1-l1_1)/(2*d);

%compute the homogeneous matrix A1 and translational vector o1 for seg1
[A1, o1]=  transcc2D(theta1);

%compute the bending angle of seg2
theta2=(l2_2-l2_1)/(2*d);

%compute the homogeneous matrix A2 and translational vector o2 for seg2
[A2, o2]=transcc2D(theta2);


% compute the tip positions of seg1 and seg2
op2=  A1 * A2 * [p02;1];
op1=  A1 *[0; 0 ;1];

% compute the orientation and postion of unit vector x axis of seg2
p=  A1 * A2 *[p2;1];


%creates a plot and calculates the positions of the end of each segment
PlotTwoSegments(theta1,theta2, op1, op2, Lsegment)

% plot(0,0,'.b'); hold on
% plot(o1(1),o1(2),'.r'); hold on
plot(p(1),p(2),'+r'); hold on
plot([op2(1) p(1)],[op2(2) p(2)],'-b'); hold on




