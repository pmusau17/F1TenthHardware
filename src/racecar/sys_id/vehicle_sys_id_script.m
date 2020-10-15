% load the data
exp3 = load_experiment_data('csvdata_29397_2.csv',1);
exp4 = load_experiment_data('csvdata_29397_3.csv',1);
exp5 = load_experiment_data('csvdata_29397_4.csv',1);


%plot_race(exp5);
% plot_race(exp8);
% plot_race(exp9);
% plot_race(exp10);
% 
% Merge the data
data_est = merge(exp4,exp5,exp3);

num_experiments = 3;


% % specify the initial parameters
%  ca = 1.1418;
%  cm = 0.0092; 
%  ch = -120.63;
%  lf = 0.225;%0.255;
%  lr = 0.225;%0.225;
%  parameters    = {ca,cm,ch,lf,lr};

% specify the initial parameters
 ca = 1.9569;
 cm = 0.0342; 
 ch = - 37.1967;
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




