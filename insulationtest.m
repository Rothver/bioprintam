%% insulationtest.m - Insulation Test Thermistor Data Logger
% This MATLAB script reads thermistor data from the Arduino insulationread.ino sketch
% and displays real-time temperature monitoring for system insulation validation.
%
% THERMISTOR CALIBRATION:
% The Arduino sketch uses centralized calibration from config/config.h:
%   - System Monitoring Sensors (Pins A2, A3): Beta coefficient = 3950K, 10K nominal
%   - Steinhart-Hart formula implementation: libraries/thermistor_sensor.h
%   - ADC Resolution: 12-bit (4095 levels, Unified across all sketches)
%   - Series Resistance: 10K ohms (configured in config.h)
%   - Purpose: Validate insulation performance during heat chamber operation
%
% OUTPUT:
% CSV files with timestamp (Time_s, Temp1_C, Temp2_C)
% Saved every 30 seconds during data collection
%
% CONNECTION:
% Ensure Arduino COM port matches your system (currently set to /dev/cu.usbmodem2101)
% Baud rate: 9600
%
% See Also: heatchamber.m, config/config.h, libraries/thermistor_sensor.h

port = "/dev/cu.usbmodem2101";  % Arduino port name
baud = 9600;
s = serialport(port, baud);

t_data = [];
temp1_data = [];
temp2_data = [];

figure('Position',[100 100 900 500]); 
hold on;

h1 = plot(nan, nan, 'r-', 'LineWidth', 2, 'DisplayName', 'Thermistor 1');
h2 = plot(nan, nan, 'b-', 'LineWidth', 2, 'DisplayName', 'Thermistor 2');

xlabel('Time (s)');
ylabel('Temperature (°C)');
title('Thermistor Temperatures Over Time', 'FontSize', 16, 'FontWeight', 'bold');
grid on;
set(gca,'FontSize',14);
ylim([20 90])
yticks(20:10:90)
yline(20:5:90,'Color',[.8 .8 .8],'HandleVisibility','off'); % Grid lines
legend([h1 h2],'Thermistor 1','Thermistor 2'); 

startTime = tic;
lastSaveTime = 0;  % Initialize last save time
saveInterval = 30; % Set save interval to 30 seconds

try
    while true  % Press Ctrl+C to stop
        if s.NumBytesAvailable > 0
            line = readline(s);
            vals = str2double(split(line, ","));
            if numel(vals)==2 && all(~isnan(vals))
                t = toc(startTime);
                t_data(end+1) = t;
                temp1_data(end+1) = vals(1);
                temp2_data(end+1) = vals(2);

                % Update plot
                set(h1, 'XData', t_data, 'YData', temp1_data);
                set(h2, 'XData', t_data, 'YData', temp2_data);
                drawnow;

                % Periodic save
                if t - lastSaveTime >= saveInterval
                    T = table(t_data', temp1_data', temp2_data', ...
                              'VariableNames', {'Time_s','Temp1_C','Temp2_C'});
                    filename = ['thermistor_data_' datestr(now,'yyyymmdd_HHMMSS') '.csv'];
                    writetable(T, filename);
                    disp(['Data saved to ' filename]);
                    lastSaveTime = t;
                end
            end
        end
    end
    
catch
    % Save final data when loop is stopped
    T = table(t_data', temp1_data', temp2_data', ...
              'VariableNames', {'Time_s','Temp1_C','Temp2_C'});
    filename = ['thermistor_data_' datestr(now,'yyyymmdd_HHMMSS') '.csv'];
    writetable(T, filename);
    disp(['Final data saved to ' filename]);
end
