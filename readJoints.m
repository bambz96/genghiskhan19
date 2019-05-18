function readJoints(serial)
    disp('Passively read joints and EE position...')
    fprintf(serial, 'R');
    i = 0;
    while i < 500
        disp(fgetl(serial));
        i = i + 1;
    end
    fprintf(serial, 'x'); % send anything to interrupt
end
