filename1 = '/media/martinspedro/Elements/mine/CAMBADA/2019-08-28 (Setup A)/Multiple LiDAR Interference/Distance/4 m/Ground Truth Model/ground_truth_bag_distance.bin'
filename2 = '/media/martinspedro/Elements/mine/CAMBADA/2019-08-28 (Setup A)/Multiple LiDAR Interference/Distance/4 m/Interference Analysis/interference_bag_distance.bin'

fileID1 = fopen(filename1);
ground_truth = fread(fileID1,'double');

fileID2 = fopen(filename2);
interference = fread(fileID2,'double');

%%
figure(2)
B = interference(interference ~= NaN);
[N, x] = hist(B/numel(B),130)
semilogy(0:1:129, N)

%%
x = 0.01:0.01:10;
inter = zeros(1, length(x));
ground =  zeros(1, length(x));

for i = 1:length(x)
inter(i) = sum(interference> x(i))/numel(interference) * 100;
ground(i) = sum(ground_truth> x(i))/numel(ground_truth) * 100;
end

%% 
figure
plot(x, abs(inter-ground))