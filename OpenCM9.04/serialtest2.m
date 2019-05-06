x = serial('COM3','BAUD',57600);

length = 1000;
data = rand(length, 4);

errors = 0;

n = 3;


tic
fopen(x);

reply = fscanf(x);
% if ~strcmp(reply, 'RDY')
%     disp('Device not ready. '+reply);
% end

for j = 1:n
    disp('Test '+string(j))
    for i = 1:length
        send = string(i) + ' ' + join(string(data(i, :)));
%         disp("sending: " + send);
        fprintf(x, send);
        reply = fscanf(x);
%         if ~strcmp(string(i), reply)
%             disp(reply);
%            errors = errors + 1; 
%         end
% %         disp(reply);
    end  
end

t = toc;
disp('Total/avg: ' + string(t) + '/' + string(t/n))
disp('Errors: ' + string(errors))
fclose(x);