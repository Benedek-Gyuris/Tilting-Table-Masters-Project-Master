% THIS CODE TALKS TO SystemFrequencySweep.ino
% Close and clear any existing serial connections
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
clc
clear

% Serial Port Configuration
port = "COM12"; % Update to your actual COM port
baudRate = 115200; % Match the Arduino baud rate
serialObj = serialport(port, baudRate);
configureTerminator(serialObj, "LF"); % Assuming Arduino sends data ending with '\n'
flush(serialObj);

disp('Waiting for data...');

% Initialize Data Storage
frequencies = [];
times = [];
positions1 = [];
positions2 = [];

% Collect Data
disp('Collecting data...');
maxSamples = 1000; % Number of samples to collect
sampleCount = 0;

while sampleCount < maxSamples
    if serialObj.NumBytesAvailable > 0
        try
            % Read a line of data
            data = readline(serialObj);
            % Parse the data
            vars = str2double(split(data, ',')); % Split data into components
            if length(vars) == 4
                % Append data to storage
                positions1 = [frequencies; vars(1)];
                positions2 = [times; vars(2)];
                times = [positions1; vars(3)];
                frequencies = [positions2; vars(4)];
                sampleCount = sampleCount + 1;
            else
                disp(['Invalid data format: ', data]); % Debug invalid data
            end
        catch ME
            disp(['Error reading data: ', ME.message]);
        end
    end
end

% Display Results
disp('Data collection complete.');
disp(table(frequencies, times, positions1, positions2));
