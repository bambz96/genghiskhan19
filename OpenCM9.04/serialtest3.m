x = serial('COM3','BAUD',57600);

% 4 polynomial coeffs
data = rand(1, 4);


tic
fopen(x);

reply = fscanf(x);
% if ~strcmp(reply, 'RDY')
%     disp('Device not ready. '+reply);
% end



send = join(string(data(1, :)));
%         disp("sending: " + send);
fprintf(x, send);
reply = fscanf(x);
%         if ~strcmp(string(i), reply)
%             disp(reply);
%            errors = errors + 1; 
%         end
% %         disp(reply);

toc
fclose(x);