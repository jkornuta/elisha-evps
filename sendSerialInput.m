clear all;

% Establish serial communication info
s = serial('/dev/tty.usbserial-AE00DRYI','BaudRate',115200,...
    'Terminator',0,'Timeout',30);

% Open connection
fopen(s);

% Set input data
Fs = 200;                   % sampling frequency [Hz]
exp_time = 10;              % desired experiment time [s]
points = Fs*exp_time;       % data points to use 
u = idinput(points,'rbs',[0,0.25]);

% Plot data
t = 0:1/Fs:exp_time-1/Fs;
plot(t,u);

% Load up command array to send to controller
% -----------------------------------
points = uint32(points);
points8 = typecast(points,'uint8');

% Inform how many data points are in the data
commands = [points8(4), points8(3), ...
    points8(2), points8(1)];

% Add on input data
commands = horzcat(commands, u');

% Add initial $ to let controller know to listen
commands = horzcat([uint8('$')], commands);

% OK, let user know sending is in progress
fprintf(1,'> Sending data...\n\n');

% Serially send commands, byte-by-byte
for i = 1:length(commands),
    while (strcmp(s.TransferStatus,'idle') == 0)
        continue;
    end;
    
    fwrite(s,commands(i),'uint8','async');
    
    % pause shortly so controller's EEPROM can record
    pause(2e-3);    % 2 ms
    
    % pause for extra second after first byte sent
    %if (i == 1)
    %    pause(2);
    %end;
end;

fprintf(1,'Data sent.\n');

% Listen for confirmation
returned = fscanf(s, '%u');
fprintf(1,'> %u received.\n\n', returned);

% Close connection
fclose(s);
