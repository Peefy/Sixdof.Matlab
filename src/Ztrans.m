function Ztrans

clc;
close all;

ZtransDuGu([1 0 0], [1 1.2 0.25])

ZtransDuGu([0 0 1], [1 1 0])

ZtransDuGu([0 0 0.25], [1 1 0.25])

ZtransDuGu([0 1 0], [1 1 0.25])

function ZtransDuGu(nums, dens)

dT = 0.047;

%���촫�ݺ���
h = tf(nums, dens)
%Z�任
zh = c2d(h, dT,'zoh')
%�õ����ӷ�ĸϵ��
[num den] = tfdata(zh, 'v');
%�õ��㼫��
[z, p, k] = tf2zpk(num, den);
