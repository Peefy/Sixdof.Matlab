function washoutfilter
close all;
clc;
global dT fs count  hpfAccWn lpfAccWn hpfAngleSpdWn inputs outputs
global IS_USE_TRANS_MATRIX IS_ADD_COOR_TURN_GAIN
global roll_scale pitch_scale yaw_scale x_scale y_scale z_scale
dT = 0.047;
fs = 1 / dT;
count = 8000;
IS_USE_TRANS_MATRIX = 1;
IS_ADD_COOR_TURN_GAIN = 0;
hpfAccWn = 1.0;
lpfAccWn = 0.1;
hpfAngleSpdWn = 0.5;
inputs = zeros(12, 3);
outputs = zeros(12, 3);
roll = zeros(count, 1);    % deg
pitch = zeros(count, 1);   % deg
yaw = zeros(count, 1);     % deg
x = zeros(count, 1);       % deg
y = zeros(count, 1);       % deg
z = zeros(count, 1);       % deg
roll_scale = 4.0;
pitch_scale = 4.0;
yaw_scale = 2.0;
x_scale = 2.0;
y_scale = 2.0; 
z_scale = 2.0;
[xacc, yacc, zacc, rollSpeed, pitchSpeed, yawSpeed, rolltxt, pitchtxt, yawtxt] = readtxt();
plotAllRecieveData(xacc, yacc, zacc, rollSpeed, pitchSpeed, yawSpeed, rolltxt, pitchtxt, yawtxt);

global accHighPassFilters_nums accHighPassFilters_dens accIntZtrans_nums accIntZtrans_dens
global accLowPassFilter_nums accLowPassFilter_dens angleHpfAndInt_nums angleHpfAndInt_dens
[accHighPassFilters_nums, accHighPassFilters_dens] = build_accHighPassFilters();
[accIntZtrans_nums, accIntZtrans_dens] = build_accIntZtrans();
[accLowPassFilter_nums, accLowPassFilter_dens] = build_accLowPassFilter();
[angleHpfAndInt_nums, angleHpfAndInt_dens] = end

figure;
plot(pitchSpeed(1:count) * 20);
hold on;
plot(pitch * pitch_scale);
hold on;
plot(pitchtxt(1:count));
title('pitch')
legend('pitchSpeed','pitchout', 'pitch')

figure;build_angleHpfAndInt();

for i = 1 : count
    i
    [x(i + 1), y(i + 1), z(i + 1), roll(i + 1), pitch(i + 1), yaw(i + 1)] = ...
        washoutfilterdo(x(i), y(i), z(i), roll(i), pitch(i), yaw(i), ...
        xacc(i), yacc(i), zacc(i), rollSpeed(i), pitchSpeed(i), rollSpeed(i));

plot(rollSpeed(1:count) * 20);
hold on;
plot(roll * roll_scale);
hold on;
plot(rolltxt(1:count));
title('roll')
legend('rollSpeed','rollout', 'roll')

figure;
hold on;
plot(xacc(1:count));
hold on;
plot(x(1:count));
title('x');
legend('xacc','x');


figure;
hold on;
plot(yacc(1:count));
hold on;
plot(y(1:count));
title('y');
legend('yacc','y');

function [x_r, y_r, z_r, roll_r, pitch_r, yaw_r] = washoutfilterdo(x, y, z, roll, pitch, yaw, xacc, yacc, zacc, rollSpeed, pitchSpeed, yawSpeed)
global IS_USE_TRANS_MATRIX IS_ADD_COOR_TURN_GAIN
global accHighPassFilters_nums accHighPassFilters_dens accIntZtrans_nums accIntZtrans_dens
global accLowPassFilter_nums accLowPassFilter_dens angleHpfAndInt_nums angleHpfAndInt_dens
acc_scale = 0.01;
angleSpd_scale = 10.0;
coor_turn_gain = 0.1;
rollrad = deg2rad(roll);
pitchrad = deg2rad(pitch);
yawrad = deg2rad(yaw);
TsMatrix = buildTsMatrix(rollrad, pitchrad, yawrad);
LsMatrix = buildLsMatrix(rollrad, pitchrad, yawrad);
fAA = [xacc * acc_scale; yacc * acc_scale; zacc * acc_scale];
wAA = [rollSpeed * angleSpd_scale; pitchSpeed * angleSpd_scale; yawSpeed * angleSpd_scale];
poses = [0; 0; 0];
f2 = [0; 0; 0];
flow = [0; 0; 0];
beta2 = [0; 0; 0];
betahigh = [0; 0; 0];
betalow = [0; 0; 0];
ahigh = [0; 0; 0];
betaS = [0; 0; 0];
if IS_USE_TRANS_MATRIX == 1
    %MatrixMultiplyVector(LsMatrix, fAA, f2);  
    f2 = LsMatrix * fAA;
	%MatrixMultiplyVector(TsMatrix, wAA, beta2);
    beta2 = TsMatrix * wAA;
else
    f2 = fAA;
    beta2 = wAA;
end
a2 = f2;
ACC_NUM = 3;
filtersindex = 1;
for i = 1 : ACC_NUM
    ahigh(i) = filtersdo(a2(i),accHighPassFilters_nums, accHighPassFilters_dens ,filtersindex); filtersindex = filtersindex + 1;
    poses(i) = filtersdo(ahigh(i),accIntZtrans_nums, accIntZtrans_dens ,filtersindex) * 1000.0; filtersindex = filtersindex + 1;
    flow(i) = filtersdo(fAA(i),accLowPassFilter_nums, accLowPassFilter_dens ,filtersindex); filtersindex = filtersindex + 1;
    betalow(i) = rad2deg(flow(i) * coor_turn_gain);
    if IS_ADD_COOR_TURN_GAIN == 1
		betaS(i) = betalow(i) + filtersdo(beta2(i),angleHpfAndInt_nums, angleHpfAndInt_dens, filtersindex); filtersindex = filtersindex + 1;
    else
        betaS(i) = filtersdo(beta2(i), angleHpfAndInt_nums, angleHpfAndInt_dens, filtersindex); filtersindex = filtersindex + 1;
    end
end
x_r = poses(1);
y_r = poses(2);
z_r = poses(3);
roll_r = betaS(1);
pitch_r = betaS(2);
yaw_r = betaS(3);

function [xacc, yacc, zacc, rollSpd, pitchSpd, yawSpd, roll, pitch, yaw] = readtxt()
data = load('illusiondata_now2.txt');
% data = load('errordata.txt');
xacc = data(1:end, 1);
yacc = data(1:end, 2);
zacc = data(1:end, 3);
rollSpd = data(1:end, 4);
yawSpd = data(1:end, 5);
pitchSpd = data(1:end, 6);
roll = data(1:end, 7);
pitch = data(1:end, 8);
yaw = data(1:end, 9);

function TsMatrix = buildTsMatrix(roll, pitch, yaw)

y = yaw;    %psi
a = roll;   %theta
b = pitch;  %phi
TsMatrix = zeros(3, 3);
TsMatrix(1, 1) = 1;
TsMatrix(1, 2) = sin(b) * tan(a);
TsMatrix(1, 3) = cos(b) * tan(a);

TsMatrix(2, 1) = 0;
TsMatrix(2, 2) = cos(b);
TsMatrix(2, 3) = -sin(b);

TsMatrix(3, 1) = 0;
TsMatrix(3, 2) = sin(b) * 1.0 / cos(a);
TsMatrix(3, 3) = cos(b) * 1.0 / cos(a);

function LsMatrix = buildLsMatrix(roll, pitch, yaw)

y = yaw;    %psi
a = roll;   %theta
b = pitch;  %phi
LsMatrix = zeros(3, 3);
LsMatrix(1, 1) = cos(a) * cos(y);
LsMatrix(1, 2) = sin(b) * cos(a) * cos(y) - cos(b) * sin(y);
LsMatrix(1, 3) = cos(b) * sin(a) * cos(y) + sin(b) * sin(y);

LsMatrix(2, 1) = cos(a) * sin(y);	
LsMatrix(2, 2) = sin(b) * sin(a) * sin(y) + cos(b) * cos(y);
LsMatrix(2, 3) = cos(b) * sin(a) * sin(y) - sin(b) * cos(y);

LsMatrix(3, 1) = -sin(a);
LsMatrix(3, 2) = sin(b) * cos(a);
LsMatrix(3, 3) = cos(b) * cos(a);

function [numsd, densd] = build_accHighPassFilters()
global fs hpfAccWn
nums = zeros(1, 3);
dens = zeros(1, 3);
nums(1) = 1; nums(2) = 0; nums(3) = 0;
dens(1) = 1; dens(2) = 2 * 1 * hpfAccWn; dens(3) = hpfAccWn * hpfAccWn;
[numsd, densd] = bilinear(nums, dens, fs);

function [numsd, densd] = build_accIntZtrans()
global fs hpfAccWn
nums = zeros(1, 3);
dens = zeros(1, 3);
nums(1) = 0; nums(2) = 0; nums(3) = 1;
dens(1) = 1; dens(2) = hpfAccWn; dens(3) = 0;
[numsd, densd] = bilinear(nums, dens, fs);

function [numsd, densd] = build_accLowPassFilter()
global fs lpfAccWn
nums = zeros(1, 3);
dens = zeros(1, 3);
nums(1) = 0; nums(2) = 0; nums(3) = lpfAccWn * lpfAccWn;
dens(1) = 1; dens(2) = 2 * 1 * lpfAccWn; dens(3) = lpfAccWn * lpfAccWn;
[numsd, densd] = bilinear(nums, dens, fs);

function [numsd, densd] = build_angleHpfAndInt();
global fs hpfAngleSpdWn
nums = zeros(1, 3);
dens = zeros(1, 3);
nums(1) = 0; nums(2) = 1; nums(3) = 0;
dens(1) = 1; dens(2) = 2 * 1 * hpfAngleSpdWn; dens(3) = hpfAngleSpdWn * hpfAngleSpdWn;
[numsd, densd] = bilinear(nums, dens, fs);

function out = filtersdo(now, inner_nums, inner_dens, filtersindex)
global inputs outputs
out = 0;

input = inputs(filtersindex, 1:end);
output = outputs(filtersindex, 1:end);

input = circshift(input',1)';
input(1) = now;
inputs(filtersindex, 1:end) = input;

for i = 1:2
    out = out - inner_dens(i + 1) * output(i);
end
for i = 1:3
    out = out + inner_nums(i) * input(i);
end

output = circshift(output',1)';
output(1) = out;
outputs(filtersindex, 1:end) = output;

function plotAllRecieveData(xacc, yacc, zacc, rollSpeed, pitchSpeed, yawSpeed, rolltxt, pitchtxt, yawtxt)
global count
figure;
plot(xacc(1:count));
hold on;
plot(yacc(1:count));
hold on;
plot(zacc(1:count));
hold on;
%plot(pitchtxt(1:count));
title('recieve xacc yacc zacc');
legend('xacc','yacc','zacc', 'pitchtxt');

figure;
plot(rollSpeed(1:count));
hold on;
plot(pitchSpeed(1:count));
hold on;
plot(yawSpeed(1:count));
hold on;
%plot(pitchtxt(1:count));
title('recieve rollSpd pitchSpd yawSpd');
legend('rollSpeed','pitchSpeed','yawSpeed', 'pitchtxt')

figure;
plot(rolltxt(1:count));
hold on;
plot(pitchtxt(1:count));
hold on;
plot(yawtxt(1:count));
title('recieve roll pitch yaw');
legend('rolltxt','pitchtxt','yawtxt');



