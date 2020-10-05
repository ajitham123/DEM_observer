function [t,real_cause,process_x,process_y,ideal_y,current,voltage,Ts,A,B,C] ...
    = DC_motor_data

filename = strcat(pwd,'\gbn_data_new_maxon_nogear');
load(filename);
% load(strcat('C:\Users\LocalAdmin\Desktop\DC_motor_data\',...
%  'motor_data\motor without gearbox\random_withoutgear_2schijf_100'));

% s = tf('s');
% H = G/s;
real_cause = actuatorgain*U';  % -actuatorgain*V'
t = t';
% ideal_y = lsim(H,real_cause,t);
process_y = Y*sensorgain;
% plot(t,process_y,t,ideal_y)
% state_sp = ss(H);

A = [0 1 0; 0 -b/J K/J; 0 -K/L -R/L];
B = [0 ;0 ;1/L];
C = [1 0 0];
[ideal_y,~,process_x] = lsim(ss(A,B,C,0),real_cause,t);
current = -I*actuatorgain;
voltage = -actuatorgain*V';

% state_sp_ = c2d(state_sp,Ts);
% process_x = zeros(size(t,2),size(state_sp_.C,2));
% for i = 2:size(t,2)
%     process_x(i,:)=(state_sp_.A*process_x(i-1,:)' + state_sp_.B*real_cause(:,i))';
% end


% load data_generation\MeasData.mat;
% 
% z = z3;
% G = tfest(z,2,0);
% t = z.SamplingInstants';
% Ts = z.Ts;
% u = z.InputData;
% process_y = z.OutputData;
% 
% state_sp = ss(G);
% A = state_sp.A;
% B = state_sp.B;
% C = state_sp.C;
% 
% [~,fit,xxx0] = compare(z,G);
% [ideal_y,~,process_x] = lsim(c2d(state_sp,Ts),u,t,xxx0);
% real_cause = u';
% fprintf('Parameter fit = %f\n',fit);


end
