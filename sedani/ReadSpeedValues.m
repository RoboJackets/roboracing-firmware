filename = 'data3.txt';
file = fopen(filename, 'r');
line = fgetl(file);
line = fgetl(file);
time = [];
speed = [];
while(line ~= -1)
    time1 = str2double(line(1:find(line == ',')));
    speed1 = str2double(line(find(line == ','):end));
    time = [time, time1];
    speed = [speed, speed1];
    line = fgetl(file);
end
fclose(file);
movingVals = find(speed~=0);
stationaryVals = find(speed==0);
endTime = movingVals(3);
startTime = movingVals(1);
time = time(startTime:endTime);
speed = speed(startTime:endTime);
time = time-time(1);

plot(time, speed);