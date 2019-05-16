function [length, xdata, ydata, zdata, thdata, gripdata] = create_all_tower_paths()

% grip and ungrip motor angles in radians
grip = 1.015976119;
ungrip = 0.422257077;

% metres, degrees, degrees, ?, seconds
% x y z theta grip duration
delay = 5;
p = [
    0.2 0 0.3 0 ungrip 2;
    0.15 -0.2 0.01 0 ungrip 7;
    0.15 -0.2 0.01 0 ungrip 8;
    0.2 0 0.01 0 ungrip 13;
    0.2 0 0.01 0 ungrip 14;
    0.125 0.25 0.01 0 ungrip 19;
    0.125 0.25 0.01 0 ungrip 20;
    0.2 0 0.01 0 ungrip 25;
    0.2 0 0.01 0 ungrip 26;
    0.15 -0.2 0.01 0 ungrip 31;
    0.15 -0.2 0.01 0 ungrip 32;
    0.2 0 0.01 0 ungrip 37;
    0.2 0 0.01 0 ungrip 42;
    0.2 0 0.3 0 ungrip 44
];
% tower corner
% x0 = 0.2;
% y0 = -0.2;
% % home pos
% xh = 0.0375;
% yh = -0.1875;
% zh = 0.003; % -0.003
% thetah = -90;
%
% p = [];
% nblocks = 3;
% i = 1; % number of loops
% n = 1; % up to block
% while n <= nblocks
%     if mod(i,2) == 0 % i = 2, 4, 6...
%         [x, y, z, theta] = block_pos(n,x0,y0);
%         n = n + 1;
%         % move to above target holding block
%         p = [p; x y z+0.05 theta grip 2];
%         % lower
%         p = [p; x y z+0.005 theta grip 1];
%         % stay at point and release block
%         p = [p; x y z+0.005 theta ungrip 0.3];
%         % raise again
%         p = [p; x y z+0.005 theta ungrip 1];
%     else
%         % move to above block pick up
%         p = [p; xh yh zh+0.05 thetah ungrip 2];
%         % move down
%         p = [p; xh yh zh thetah ungrip 2];
%         % nudge forwards to fit block precisely and pick up
%         p = [p; xh+0.001 yh zh thetah grip 0.3];
%         % move up
%         p = [p; xh yh zh+0.05 thetah grip 1];
%     end
%     i = i + 1;
% end
% p = [p; xh yh zh+0.05 thetah ungrip 2];

disp(p)

[length, ~] = size(p);
length = length - 1;

xdata = zeros(length, 5);
ydata = zeros(length, 5);
zdata = zeros(length, 5);
thdata = zeros(length, 5);
gripdata = zeros(length, 5);

%  current to p(1), the first pose, in 3 seconds

for i = 1:length
    x0 = p(i,1); xf = p(i+1,1);
    y0 = p(i,2); yf = p(i+1,2);
    z0 = p(i,3); zf = p(i+1,3);
    th0 = p(i,4)*pi/180; thf = p(i+1,4)*pi/180;
    gr0 = p(i,5); grf = p(i+1,5);
    tf = p(i,6);
    xdata(i,:) = cubic_coeffs(x0, 0, xf, 0, tf);
    ydata(i,:) = cubic_coeffs(y0, 0, yf, 0, tf);
    zdata(i,:) = cubic_coeffs(z0, 0, zf, 0, tf);
    thdata(i,:) = cubic_coeffs(th0, 0, thf, 0, tf);
    gripdata(i,:) = cubic_coeffs(gr0, 0, grf, 0, tf);
end


end
