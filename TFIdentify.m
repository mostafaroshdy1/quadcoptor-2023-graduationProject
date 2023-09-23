% load your data (text file with your two columns)
load SystemData.txt;

% sample index, reducing of input to get single step instead of square wave
x = 1:1:length(SystemData);
dataSize = length(SystemData);
in = 0.01 * ones(dataSize,1);
SystemData(x > 2900,:)=[];
x(x > 2900)=[];

% plot data
figure(1)
plot(x,SystemData(:,1)); hold on
%plot(x,data(:,2)); hold off

% prepare data for tftest, 100 is a random chosen sampling time
tfdata = iddata(SystemData(:,1),in,0.01);

% estimate system, factor 5 -> number of poles (variable as desired)
sys = tfest(tfdata,2,0)

% plot step response (factor 5 comes from input)
figure(2)
step(2*sys)