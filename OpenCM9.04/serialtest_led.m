% simple serial command of led

x = serial('COM4','BAUD',57600);

go = 1; 

while go
    a = input('Press 1 to turn ON LED & 0 to turn OFF:');
    fopen(x);
    fprintf(x,a);
    fclose(x); 
    if a == 2
        go = 0; 
    end
end