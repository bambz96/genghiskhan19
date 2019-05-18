towerHeightMin = 15/2;
towerHeightMax = 270-15/2;

jenghis = robot; 

radii = 200:10:300;
n = length(radii);

T_bot = zeros(1,n);
T_top = zeros(1,n);

for i = 1:n
    T_bot(i) = (jenghis.torque.findStaticMotorTorque(radii(i),0,towerHeightMin,0));
    T_top(i) = (jenghis.torque.findStaticMotorTorque(radii(i),0,towerHeightMax,0));
end

plot(radii,T_bot,radii,T_top)
