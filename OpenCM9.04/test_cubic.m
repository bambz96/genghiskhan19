t = 0:0.01:1;
[x, xd] = cubic(t, 0.25, 0, 0.037, 0);

hold on
plot(t, x)

[x, xd] = cubic(t, 0.037, 0, 0.25, 0);

plot(t, x)