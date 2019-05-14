close all
%% robot dimensions
dim.h = 0.20625;
dim.l1 = 0.2;
dim.l2 = 0.2;
dim.l3 = 0.1;
dim.l4 = 0.01;

tests = 500;

e = zeros(5, tests);

doDraw = 0;
randomJoints = 1;

t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;
t5 = 0;

for i = 1:4
    for j = 1:tests
        if randomJoints
            t1 = 0;
            t2 = randRange(30,90);
            t3 = randRange(-10,-70);
            t4 = randRange(-10, -120);
        end
        te = zeros(1,4);
        te(i) = 2*rand();
        vE = draw_robot(t1, t2, t3, t4, t5, dim, 0, 30, 30, 0.8);
        vEe = draw_robot(t1+te(1), t2+te(2), t3+te(3), t4+te(4), t5, dim, doDraw, 0, 0, 0.8);
        e(i,j) = norm(vE - vEe);
    end
end

norms = [norm(e(1,:), inf) norm(e(2,:), inf) norm(e(3,:), inf) norm(e(4,:), inf)];
means = [mean(e(1,:)) mean(e(2,:)) mean(e(3,:)) mean(e(4,:))];

subplot(121)
bar(norms.*1000)
ylabel('Max error (mm)')

subplot(122)
bar(means.*1000)
ylabel('Mean error (mm)')

% histogram(e(1,:))
% histogram(e(2,:))
% histogram(e(3,:))
% histogram(e(4,:))
% legend('q1', 'q2', 'q3', 'q4')

function r = randRange(a, b)
r = (b-a).*rand() + a;
end