t = -1:0.01:1;
[x, xd] = cubic(t, 0.5, 0, 0, 0);

hold on
plot(t, x)

[x, xd] = cubic(t, 0, 0, 1, 0);

plot(t, x)