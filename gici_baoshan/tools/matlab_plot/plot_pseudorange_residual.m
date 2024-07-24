clear; clc;

%% Read
path = '/home/cc/datasets/log_estimator/pseudorange_residual-20230307-082110.log';
fp = fopen(path, 'r');
max_channel = 200;
length_channel = 0;
data = cell(max_channel, 1);
prns = strings(max_channel, 1);
codes = strings(max_channel, 1);
data_lengths = zeros(max_channel, 1);
initial_time = 0;
end_time = 0;
for i = 1 : max_channel
    data{i} = zeros(3600 * 6, 2);
end
while ~feof(fp)    
    line = fgetl(fp);                                 
    splitLine = strsplit(line);

    if line(1) == '>'
        time = str2double(splitLine(8));
        if initial_time == 0
            initial_time = time;
        end
        end_time = time;
    else
        prn = splitLine(1); prn = prn{1};
        for i = 2 : size(splitLine, 2)
            if mod(i, 2) == 0
                code = splitLine(i); code = code{1};
            else
                value = str2double(splitLine(i));
                index = 0;
                for j = 1 : max_channel
                    if prn == prns(j) && code == codes(j)
                        index = j;
                        break;
                    end
                end
                if (index == 0)
                    length_channel = length_channel + 1;
                    prns(length_channel) = prn;
                    codes(length_channel) = code;
                    index = length_channel;
                end
                data_lengths(index) = data_lengths(index) + 1;
                data{index}(data_lengths(index), 1) = time;
                data{index}(data_lengths(index), 2) = value;
            end
        end
    end
end
fclose(fp);

%% Plot
plot_system = "G";
plot_code = "2S";
plot_y_range = 1.5;
% plot_prn = "G01";

figure;
legends = cell(1, 1);
num_plotted = 0;
for i = 1 : length_channel
    prn = prns(i);
    code = codes(i);
    if (contains(prn, plot_system) == 0 || strcmp(code, plot_code) == 0) 
        continue;
    end
%     if (strcmp(plot_prn, prn) == 0)
%         continue;
%     end
    plot(data{i}(1:data_lengths(i),1) - initial_time, data{i}(1:data_lengths(i),2), '.');
    hold on;

    num_plotted = num_plotted + 1;
    legends{num_plotted} = prns(i);
end
if num_plotted > 0, legend(legends); end
grid on;
xlim([0, end_time - initial_time]);
ylim([-plot_y_range, plot_y_range]);

