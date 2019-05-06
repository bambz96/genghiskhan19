x = serial('/dev/cu.usbmodem14101','BAUD',57600);

go = 1;

while go
    a = input('Press 1 to turn ON LED & 0 to turn OFF:');
    fopen(x);
    tic
    fprintf(x,a);
    toc
    disp(fscanf(x));
    fclose(x); 
    if a == 2
        go = 0; 
    end
end
