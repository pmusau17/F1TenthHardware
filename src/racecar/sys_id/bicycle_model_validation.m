% validate the sysid model returned from the grey-box system identification
% specify the parameters obtained from the sys-id in the file
% simulate_bicycle_euler.m
mse = validate_experiment('csvdata_20774_0.csv');
mse2 = validate_experiment('csvdata_20774_1.csv');
mse3 = validate_experiment('csvdata_20774_2.csv');
mse4 = validate_experiment('csvdata_20774_3.csv');
mse5 = validate_experiment('csvdata_21100_0.csv');

mses = [mse,mse2,mse3,mse4,mse5];
avg_mse= mean(mses);