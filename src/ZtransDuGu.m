function ZtransDuGu(nums, dens)

dT = 0.01;

%���촫�ݺ���
h = tf(nums, dens)
%Z�任
zh = c2d(h, dT,'zoh')
%�õ����ӷ�ĸϵ��
[num den] = tfdata(zh, 'v');
%�õ��㼫��
[z, p, k] = tf2zpk(num, den);
