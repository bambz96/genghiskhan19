close all
for i = 1:6
    hold on
    axis([0 0.3 0 0.3])
    [x, y, z, theta] = block_pos(i, 0.1, 0.1);
    scatter3(x,y,z)
    pause(0.3)
end