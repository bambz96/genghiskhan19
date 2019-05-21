function toggleDebugging(serial)
    global debugging
    fprintf(serial, "D");
    received = strtrim(fscanf(serial));
    if strcmp(received, "D")
        debugging = ~debugging;
        disp('Toggled debugging, current state: '+string(debugging))
    else
        disp('Device did not respond correctly: '+string(received))
    end
end