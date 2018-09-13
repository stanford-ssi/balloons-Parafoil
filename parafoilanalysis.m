% Parafoil Launch 5/19/18

filename = 'parafoildata.xlsx';
sheet = 'sheet1';

altRange = 'C19951:C20285';
latRange = 'I19951:I20285';
lonRange = 'J19951:J20285';

alt = xlsread(filename,sheet,altRange);
lat = xlsread(filename,sheet,latRange);
lon = xlsread(filename,sheet,lonRange);

%average = [1 1 1];
%alt = conv(average, alt);
%lat = conv(average, lat);
%lon = conv(average, lon);
figure();
plot3(lat,lon,alt,'b');
grid on;
xlabel('Latitude');
ylabel('Longitude');
zlabel('Altitude');
title('Parafoil Launch 1 Data');

%%

filename = 'parafoildata.xlsx';
sheet = 'sheet1';

altRange = 'C24277:C25489';
latRange = 'I24277:I25489';
lonRange = 'J24277:J25489';

alt = xlsread(filename,sheet,altRange);
lat = xlsread(filename,sheet,latRange);
lon = xlsread(filename,sheet,lonRange);

% figure();
% plot3(lat,lon,alt,'b');
% grid on;
% xlabel('Latitude');
% ylabel('Longitude');
% zlabel('Altitude');
% title('Parafoil Launch 2 Data');
figure();
plot(lon,lat);
xlabel('longitude');
ylabel('latitude');

%% Parfoil Launch 2 time vs altitude


filename = 'parafoildata.xlsx';
sheet = 'sheet1';

altRange = 'C24270:C25363';
time = 'A24270:A25363';
figure();
alt = xlsread(filename,sheet,altRange);
time = xlsread(filename,sheet,time);
time = time/1000;
plot(time,alt,'b');
grid on;

xlabel('Time');
ylabel('Altitude');
title('Parafoil Launch 2 Altitude vs Time Data');

%% Parfoil Launch 1 time vs altitude


filename = 'parafoildata.xlsx';
sheet = 'sheet1';

altRange = 'C19951:C20285';
time = 'A19951:A20285';
figure();
alt = xlsread(filename,sheet,altRange);
time = xlsread(filename,sheet,time);
time = time/1000;
plot(time,alt,'b');
grid on;

xlabel('Time');
ylabel('Altitude');
title('Parafoil Launch 1 Altitude vs Time Data');
