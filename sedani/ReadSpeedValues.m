filename = 'speeds.txt';
file = fopen(filename, 'r');
line = fgetl(file);
time = [];
speed = [];
while(line ~= -1)
    time1 = str2double(line(1:find(line == ' ')));
    speed1 = str2double(line(find(line == ' '):end));
    time = [time, time1];
    speed = [speed, speed1];
    line = fgetl(file);
end
fclose(file);

plot(speed, time);