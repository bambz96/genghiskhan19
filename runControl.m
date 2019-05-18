function success = runControl(serial, type, chunk_i)
    % type is "PC" or "VC"
    disp('Run control on stored trajectories...')
    fprintf(serial, type);
    received = strtrim(fscanf(serial));
    if strcmp(received, type)
        if strcmp(type, "PC")
            disp('Beginning position control.')
        elseif strcmp(type, "VC")
            disp('Beginning velocity control.')
        end
    else
        disp('Device did not respond correctly: ' + join(string(received)))
    end

    % assume success, set false (0) if
    success = 1;

    global debugging
    if debugging
        % will read debug output until receives "DONE" from device
        if strcmp(type, "PC")
            readPositionControlDebugging(serial)
        elseif strcmp(type, "VC")
            readVelocityControlDebugging(serial)
        end
    else
        % wait for device to confirm position control completed planned trajectories
        while get(serial, 'BytesAvailable') == 0
        end

        received = strtrim(fscanf(serial));
        if strcmp(received, "DONE")
            if nargin == 2 % no chunk_i input
                disp('Path completed.')
            elseif nargin == 3
                disp('Chunk '+string(chunk_i)+' completed.')
            end
        else
            disp('Device did not respond correctly: ' + received)
            disp('Stopping operation.')
        end
    end
end