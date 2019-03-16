clc;
close all;
hz = 0.651;
chirpTime = 2.5;
val = 0.2;
t = 0 : 0.00095 :  20.775;
count = length(t);
f = 0 : 0.00095 :  20.775;
y = 0 : 0.00095 :  20.775;
stopTime = 9;
index = 0;
for i = -1 : 100
    if pi * i + 0.5 * pi >= (stopTime - chirpTime) * 2 * pi * hz
        index = i;
        break;
    end
end
finalstopTime = (pi * index + 0.5 * pi) / (2 * pi * hz) + chirpTime - 1;
for k = 1 : count;
    if t(k) <= chirpTime
        f(k) = t(k) * hz / chirpTime / chirpTime;
        y(k) = val * sin(2 * pi * f(k) * t(k));
    else
        if t(k) <= finalstopTime
            y(k) = val * sin(2 * pi * hz * (t(k) - chirpTime) + 2 * pi * hz);
        else
            y(k) = 0;
        end
    end
end

plot(t, y)
