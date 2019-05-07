% successfully sends polynomial coefficients

serial = serial('COM3','BAUD',57600);

%% 4 polynomial coeffs
length = 1;
% data = rand(length, 4);
[a3, a2, a1, a0] = cubic(0, 0, 1, 1, 0.5);
data = [a3 a2 a1 a0];

%% open serial and wait to establish
fopen(serial);
pause(1.5);

%% send number of rows
tic
fprintf(serial, string(length));
reply = strtrim(fscanf(serial));
if ~strcmp(join(string(reply)), string(length))
    disp('Device did not agree on length, will not send. '+join(string(reply)));
    fclose(serial);
    delete(serial);
    return;
else
    disp('Device agreed on length of '+string(length)+'.')
end
toc

%% send rows
tic
for i = 1:length
    send = join(string(data(i, :)));
    disp("send: " + send);
    fprintf(serial, send);
    reply = strtrim(fscanf(serial));
    % if ~strcmp(string(i), reply)
    %     disp(reply);
    %    errors = errors + 1; 
    % end
    disp("recv: " + reply);
end
toc

%% received plotting data
idx = [];
t = [];
x = [];
running = 1;
while running < 100
    data = strtrim(fscanf(serial));
    res = regexp(data, '[+-]?\d+\.?\d*','match');
    idx = [idx str2double(res{1})];
    t = [t str2double(res{2})];
    x = [x str2double(res{3})];
    running = running + 1;
end
plot(t, x)
%% clean up
fclose(serial);
delete(serial);