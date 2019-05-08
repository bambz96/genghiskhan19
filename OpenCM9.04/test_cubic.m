t = 0:0.01:1;
[x, xd] = cubic(t, 0.1, 0, 0.2, 0);

plot(t, x)