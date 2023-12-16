myx = out.x;
myy = out.y;

t = linspace(0,10,1001);
z = sqrt(myx.^2 + myy.^2);

figure();
plot(myx,myy)
%plot(t,myx)

%% matlab test 
clear all;
vr = 1;
vl = 0.5;

V = (vr+vl)/2;
L = 0.17;

omega = (vr-vl)/L;

u = [vr;vl];
C = [1 0 0;0 1 0;0 0 1];

x(1:3,1) = [0;0;0];
t = 0:0.01:10;

for i = 2:size(t,2)
    x(:,i) = x(:,i-1) + 0.01*[V*cos(x(3,i-1));V*sin(x(3,i-1));(vr-vl)/L];
end

plot(t,x(1:2,:))
legend();