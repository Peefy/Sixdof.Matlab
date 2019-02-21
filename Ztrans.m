function Ztrans

clc;
close all;

ZtransDuGu([1 0 0], [1 1.2 0.25])

ZtransDuGu([0 0 1], [1 1 0])

ZtransDuGu([0 0 0.25], [1 1 0.25])

ZtransDuGu([0 1 0], [1 1 0.25])

function ZtransDuGu(nums, dens)

dT = 0.047;

%构造传递函数
h = tf(nums, dens)
%Z变换
zh = c2d(h, dT,'zoh')
%得到分子分母系数
[num den] = tfdata(zh, 'v');
%得到零极点
[z, p, k] = tf2zpk(num, den);
