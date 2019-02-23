function R = BuildConversionMatrix(yaw, roll, pitch)
c = yaw;
a = roll;
b = pitch;
R = [cos(c) * cos(b), -sin(c) * cos(a) + cos(c) * sin(b) * sin(a), sin(c) * sin(a) + cos(c) * sin(b) * cos(a);
        sin(c) * cos(b), cos(c) * cos(a) + sin(c) * sin(b) * sin(a), -cos(c) * sin(a) + sin(c) * sin(b) * cos(a);
        -sin(b), cos(b) * sin(a), cos(b) * cos(a)];