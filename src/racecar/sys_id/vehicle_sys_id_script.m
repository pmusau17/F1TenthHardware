% load the data
exp1 = load_experiment_data('csvdata_18277_0.csv',1);
exp2 = load_experiment_data('csvdata_18277_1.csv',1);
exp3 = load_experiment_data('csvdata_18277_3.csv',1);
exp4 = load_experiment_data('csvdata_18277_4.csv',1);
exp5 = load_experiment_data('csvdata_18277_6.csv',1);
exp6 = load_experiment_data('csvdata_18277_7.csv',1);

exp7 = load_experiment_data('csvdata_22648_0.csv',1);
exp8 = load_experiment_data('csvdata_22648_1.csv',1);
exp9 = load_experiment_data('csvdata_22648_2.csv',1);
exp10 = load_experiment_data('csvdata_22648_3.csv',1);

%plot_race(exp7);
%plot_race(exp8);
%plot_race(exp9);
%plot_race(exp10);

% Merge the data
data_est = merge(exp1,exp2,exp3,exp4,exp5,exp6,exp7,exp8,exp9,exp10);

num_experiments = 10;


% specify the initial parameters
 ca = 1.1418;
 cm = 0.0092; 
 ch = -120.63;
 lf = 0.225;%0.255;
 lr = 0.225;%0.225;
 parameters    = {ca,cm,ch,lf,lr};

 % SysID options 
opt = nlgreyestOptions;
opt.Display = 'Full';
opt.SearchOptions.FunctionTolerance = 1e-7;
opt.SearchOptions.MaxIterations = 100;

% not sure why they call it order
% it species the number of model outputs
% the model inputs, and states
% so for us its 2 inputs, 4 outputs, 4 states
order         = [4 2 4];
Ts = 0;

% Let's try the system identification now
for i=1:num_experiments
    
% get the dataset   
ds = getexp(data_est,i);
initial_states = reshape(ds.y(1,:),[],1);
nonlinear_model = idnlgrey('bicycle_model',order,parameters,initial_states,Ts);
nonlinear_model.algorithm.SimulationOptions.Solver = 'ode45';
nonlinear_model.algorithm.SimulationOptions.MaxStep = 1e-1;
nonlinear_model.algorithm.SimulationOptions.InitialStep = 1e-4;
nonlinear_model.SimulationOptions.AbsTol = 1e-6;
nonlinear_model.SimulationOptions.RelTol = 1e-5;
setpar(nonlinear_model,'Fixed',{false, false,false,true,true});


nonlinear_model = nlgreyest(ds,nonlinear_model,opt);

params = nonlinear_model.Parameters;
params = [params.Value];
parameters = {params(1),params(2),params(3),lf,lr};


figure();
compare(ds,nonlinear_model)
end


close all 
% Let's see how the parameters do overall. 


for i=1:num_experiments
% get the dataset   
ds = getexp(data_est,i);
initial_states = reshape(ds.y(1,:),[],1);
nonlinear_model = idnlgrey('bicycle_model',order,parameters,initial_states,Ts);
figure();
compare(ds,nonlinear_model)
end 




