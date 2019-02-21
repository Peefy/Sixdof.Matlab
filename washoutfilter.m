function washoutfilter
close all;
clc;
global dT count
dT = 0.047;
count = 9000;
[xacc, yacc, zacc, rollSpd, pitchSpd, yawSpd, roll, pitch, yaw] = readtxt();
plot(xacc);


function washoutfilterdo(x, y, z, roll, pitch, yaw, xacc, yacc, zacc, rollSpd, pitchSpd, yawSpd)



function [xacc, yacc, zacc, rollSpd, pitchSpd, yawSpd, roll, pitch, yaw] = readtxt()
data = load('illusiondata.txt');
xacc = data(1:end, 1);
yacc = data(1:end, 2);
zacc = data(1:end, 3);
rollSpd = data(1:end, 4);
pitchSpd = data(1:end, 5);
yawSpd = data(1:end, 6);
roll = data(1:end, 7);
pitch = data(1:end, 8);
yaw = data(1:end, 9);

function Ts = buildTsMatrix(roll, pitch, yaw)

function Ls = buildLsMatrix(roll, pitch, yaw)

function [nums, dens] = bilinear(b, a, n, fs)
