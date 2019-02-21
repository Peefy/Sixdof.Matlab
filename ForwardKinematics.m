function [xx, yy, zz, roll, yaw, pitch] = ForwardKinematics(dLen1, dLen2, dLen3, dLen4, dLen5, dLen6)

UpZ = 100;
DownZ = 700;
UpR = 680;
DownR = 840;
Dis = 190;

if nargin == 1
    dlens = dLen1;
    dLen1 = dlens(1);
    dLen2 = dlens(2);
    dLen3 = dlens(3);
    dLen4 = dlens(4);
    dLen5 = dlens(5);
    dLen6 = dlens(6);
end

f1 = @(x, y, z, a, b, c)((y + UpZ*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/2) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*cos(b)*sin(c))^2 + (UpZ*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - x + DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/2) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) - UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*cos(b)*cos(c))^2 + (DownZ + z - UpZ*cos(a)*cos(b) - UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*sin(b) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*cos(b)*sin(a))^2)^(1/2) - dLen1 - ((UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6) - DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/2))^2 + (DownZ - UpZ)^2 + (UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/2))^2)^(1/2);
f2 = @(x, y, z, a, b, c)((UpZ*cos(a)*cos(b) - z - DownZ + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*sin(b) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*cos(b)*sin(a))^2 + (x - UpZ*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/3) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*cos(b)*cos(c))^2 + (y + UpZ*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) - DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/3) - UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*cos(b)*sin(c))^2)^(1/2) - dLen2 - ((DownZ - UpZ)^2 + (UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/3))^2 + (UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6) + DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/3))^2)^(1/2);
f3 = @(x, y, z, a, b, c)((UpZ*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - x + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - (2*pi)/3) - UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*cos(b)*cos(c))^2 + (DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - (2*pi)/3) - UpZ*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) - y + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*cos(b)*sin(c))^2 + (DownZ + z - UpZ*cos(a)*cos(b) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*sin(b) - UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*cos(b)*sin(a))^2)^(1/2) - dLen3 - ((DownZ - UpZ)^2 + (UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - (2*pi)/3))^2 + (UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2) + DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - (2*pi)/3))^2)^(1/2);
f4 = @(x, y, z, a, b, c)((UpZ*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - x + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - (2*pi)/3) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*cos(b)*cos(c))^2 + (DownZ + z - UpZ*cos(a)*cos(b) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*sin(b) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*cos(b)*sin(a))^2 + (y + UpZ*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - (2*pi)/3) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2)*cos(b)*sin(c))^2)^(1/2) - dLen4 - ((DownZ - UpZ)^2 + (UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - (2*pi)/3))^2 + (UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/2) + DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - (2*pi)/3))^2)^(1/2);
f5 = @(x, y, z, a, b, c)((DownZ + z - UpZ*cos(a)*cos(b) - UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*sin(b) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*cos(b)*sin(a))^2 + (x - UpZ*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/3) - UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*cos(b)*cos(c))^2 + (y + UpZ*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/3) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6)*cos(b)*sin(c))^2)^(1/2) - dLen5 - ((DownZ - UpZ)^2 + (UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/3))^2 + (UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - (5*pi)/6) + DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/3))^2)^(1/2);
f6 = @(x, y, z, a, b, c)((y + UpZ*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) - DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/2) - UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*cos(b)*sin(c))^2 + (UpZ*cos(a)*cos(b) - z - DownZ + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*sin(b) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*cos(b)*sin(a))^2 + (x - UpZ*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/2) + UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6)*cos(b)*cos(c))^2)^(1/2) - dLen6 - ((UpR*cos(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6) - DownR*cos(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/2))^2 + (DownZ - UpZ)^2 + (UpR*sin(acos((Dis^2 - 2*UpR^2)/(2*UpR^2))/2 - pi/6) + DownR*sin(acos((Dis^2 - 2*DownR^2)/(2*DownR^2))/2 - pi/2))^2)^(1/2);
f = @(x)[
f1(x(1), x(2), x(3), x(4), x(5), x(6))
f2(x(1), x(2), x(3), x(4), x(5), x(6))
f3(x(1), x(2), x(3), x(4), x(5), x(6))
f4(x(1), x(2), x(3), x(4), x(5), x(6))
f5(x(1), x(2), x(3), x(4), x(5), x(6))

f6(x(1), x(2), x(3), x(4), x(5), x(6))
];
x = fsolve(f, [0 0 0 0 0 0]);
xx = x(1);
yy = x(2);
zz = x(3);
roll = x(4);
pitch = x(5);
yaw = x(6);




