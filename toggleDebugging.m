function toggleDebugging(serial)
    global debugging
    fprintf(serial, "D");
    received = strtrim(fscanf(serial));
    if strcmp(received, "D")
        debugging = ~debugging;
        disp('Toggled debugging, current state: '+debugging)
    else
        disp('Device did not respond correctly: '+received)
    end
end