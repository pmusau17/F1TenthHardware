function plot_race(data)
% Let's look at the postion data we have
time = data.SamplingInstants;

figure();
plot(data.y(:,1),data.y(:,2));
xlabel('x (m)') 
ylabel('y (m)') 
title("Position of Vehicle")

% Let's look at the speed and throttle data we have 
figure();
plot(time,data.y(:,3),'DisplayName','speed');
hold on;
plot(time, data.u(:,1),'DisplayName','throttle');
xlabel('Time (s)') 
ylabel('Speed (m/s)') 
title("Speed input of the car")
legend

% Let's look at the angle and input data we have 
figure();
plot(time, data.y(:,4));
hold on;
plot(time, data.u(:,2));
xlabel('Time (s)') 
ylabel('theta (rad)') 
title("Heading of car")
end

