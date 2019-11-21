clear all; 
close all;

% expe = 4; % use for loading data, exp = 4 gives the data for standing still
% subexp = 2; % we did 2 experiments for this, pick 1 or 2 
% 
% load(char("C:\Users\LocalAdmin\Desktop\BEP_data\MATfiles\matdata"+expe+"."+subexp+".mat"));% load matfiles
% s_est = estimate_smoothness(Y(4000:4150)',0:.1:15);
% UIO_estimator;

if_dataset = 0;
if if_dataset==1
    [Tt,model.real_cause,model.process_x,model.process_y,ideal_y,current,...
        voltage,model.sam_time,model.A,model.B,model.C] = DC_motor_data;
    model.ideal_x = model.process_x;
    model.t_end = Tt(end) + model.sam_time;
    brain.ny = size(model.C,1);
    model.t = Tt + model.sam_time;
else
    model.sam_time = .1;
    model.t_end = 32;
    Tt  = 0:model.sam_time:model.t_end-model.sam_time;
    model.t = Tt + model.sam_time;
    
    model.real_cause = exp(-.25*(model.t-12).^2);
%         model.real_cause = sin(.25*model.t);
%         model.real_cause = (1/model.t_end)*(model.t);
    model.A = [-.25 1; -.5 -.25];
    model.B = [1; 0];
    model.C = [.125 .1633; .125 .0676; .125 -.0676; .125 -.1633];
    
%     k =1; m = 1; b = -.1;  % b_critical = 2 sqrt(k*m)
%     model.A = [0 1; -k/m -b/m];
%     model.B = [0; 1/m];
%     model.C = [1 0];
end

brain.nt = size(model.t,2);
if_predict_y = 0;
if_UIO = 0;
UIO_gamma = 125;%3050; %motor
if_cause = 1;   % should model with known causes?

iter = 1;
% s_est = zeros(1,iter);
% s_realA = zeros(1,iter);
UIO_gains = [1200 1200 1200 1400 1500];

% figure(1); plot([0,10],[0,10],'b'); hold on;

for i=1:iter
    for j = 1:1        
        for k = 1:1     % number of trials

% UIO_gamma = UIO_gains(i);

% if j==1
%     model.real_cause = exp(-.25*(model.t-12).^2); col = 'r';
% elseif j==2
%     model.real_cause = sin(.25*model.t); col = 'g';
% elseif j==3
%     model.real_cause = (1/model.t_end)*(model.t); col = 'b';
% end
% 
% while 1
%     brain.ny = randi([2,5],1); brain.nx = 2;
%     model.A = -1+2*rand(brain.nx); model.B = -1+2*rand(brain.nx,1); 
%     model.C = -1+2*rand(brain.ny,brain.nx);
%     if isstable(ss(model.A,model.B,model.C,0)) 
%         break; 
%     end
% end

model.s = .5; %0.01 + exp(-16); 
brain.s =  model.s;%0.01 + exp(-16);    % motor 

model.p = 6;
model.d = 2;
brain.p = 6; % motor data 5
brain.d = 2; % motor data 2

model.sigma_w = exp(-4);%exp(-6);     %motor data
model.sigma_z = exp(-4);%exp(-13);  %motor data
brain.sigma_v = exp(4);
brain.sigma_w = model.sigma_w;
brain.sigma_z = model.sigma_z;%exp(-4);

[model, brain] = generative_process(model,brain,if_dataset);
brain.Y_embed = generalized_process(model.process_y,model.t,model.sam_time,...
    brain.ny,brain.p,brain.d);

output = observer(model,brain,if_UIO,UIO_gamma,if_cause,if_dataset,if_predict_y);

% Error in estimation
if_t_trim = 1;      % 1: trim out estimates at both ends (inaccurate derivatives)
if if_t_trim == 1
    t_trim = brain.p+2:brain.nt-brain.p-2;
else 
    t_trim = 1:brain.nt;
end

SSE.DEM.x(j,k) = (sum(sum((output.DEM_x(t_trim,1:brain.nx)-model.ideal_x(t_trim,:)).^2)));
SSE.kalman.x(j,k) = (sum(sum((output.kalman_x(:,t_trim)'-model.ideal_x(t_trim,:)).^2)));
SSE.DEM.v(j,k) = (sum(sum((output.DEM_x(t_trim,brain.nx*(brain.p+1)+1)-model.real_cause(:,t_trim)').^2)));
if if_UIO == 1
    SSE.UIO.x(j,k) = sum(sum((output.UIO_x_est(:,t_trim)-model.ideal_x(t_trim,:)').^2));
    SSE.UIO.v(j,k) = sum((output.UIO_v_est(:,t_trim)-model.real_cause(:,t_trim)).^2);
end
if if_cause == 1
    SSE.DEMv.x(j,k) = (sum(sum((output.DEMv_x(t_trim,1:brain.nx)-model.ideal_x(t_trim,:)).^2)));
    SSE.kalmanv.x(j,k) = (sum(sum((output.kalmfv_x(:,t_trim)'-model.ideal_x(t_trim,:)).^2)));
end
        print_results(SSE,j,k,if_UIO,if_cause);
        
% figure(1);plot(SSE.kalmanv.x(j,k),SSE.DEMv.x(j,k),strcat(col,'*')); 
% ylim([0,10]); xlim([0,10]); hold on; 

        end
        %         SSE_mean_x(i,:) = mean([SSE.kalman.x' SSE.UIO.x' SSE.DEM.x' ...
%             SSE.kalmanv.x' SSE.DEMv.x']);
%         SSE_std_x(i,:)  = std( [SSE.kalman.x' SSE.UIO.x' SSE.DEM.x' ...
%             SSE.kalmanv.x' SSE.DEMv.x']);
%         SSE_mean_v(i,:) = mean([SSE.UIO.v' SSE.DEM.v']);
%         SSE_std_v(i,:)  = std( [SSE.UIO.v' SSE.DEM.v']);
        
    end
%     SSE_mean_x(i,:) = mean([SSE.kalman.x(1,:); SSE.DEM.x],2)'; 
%     SSE_std_x(i,:) = std([SSE.kalman.x(1,:); SSE.DEM.x],0,2)';
%     SSE_mean_v(i,:) = mean(SSE.DEM.v,2)'; SSE_std_v(i,:) = std(SSE.DEM.v,0,2)';

end
%%
% figure; hBar = bar(SSE_mean_x); hold on; 
% errorbar(bsxfun(@plus, hBar(1).XData, [hBar.XOffset]')',SSE_mean_x,SSE_std_x,'k.')
% xticklabels({'\sigma=0.1','\sigma=0.3','\sigma=0.5','\sigma=0.7','\sigma=0.9'})
% ylabel('SSE of state estimation');
% % legend({'p = 0','p = 1','p = 2','p = 3','p = 4','p = 5','p = 6'},...
% %     'Orientation','horizontal','NumColumns',4);
% % legend('KF: unknown input','UIO: unknown input','DEM: unknown input',...
% %     'KF: known input','DEM: known input');
% legend('KF','DEM(\sigma=0)','DEM(\sigma=\sigma)');
% set(findall(gcf,'-property','FontSize'),'FontSize',12)

%%
% print_results(SSE,if_UIO,if_cause);
plot_results(output,model,brain,if_UIO,if_cause);


% figure;  plot(1*ones(1,iter),SSE_x_kalman,'b*');
% hold on; plot(2*ones(1,iter),SSE_x_UIO,'c*');
% hold on; plot(3*ones(1,iter),SSE_x_DEM,'g*'); 
% hold on; plot(4*ones(1,iter),SSE_x_kalmv,'r*','MarkerSize',5); 
% hold on; plot(5*ones(1,iter),SSE_x_DEMv,'m*');
% 
% xlim([0,6]); set(gca,'XTick',[]); legend('KF','UIO','DEM','KF\_v','DEM\_v');

% figure; plot(1*ones(1,iter),SSE_v_UIO,'r*');
% hold on; plot(2*ones(1,iter),SSE_v_DEM,'b*'); 
% xlim([0,3]); set(gca,'XTick',[]); legend('UIO\_cause','DEM\_cause');

%%
% load('C:\Users\LocalAdmin\Desktop\DEM_test\DEM_Y_friston.mat');
% load('C:\Users\LocalAdmin\Desktop\DEM_test\DEM_pX_friston.mat');
% load('C:\Users\LocalAdmin\Desktop\DEM_test\DEM_Xest_friston.mat');
% figure; plot(process_x,'-'); hold on; plot(DEM_pX_friston','-.'); 
% figure; plot(process_y,'-'); hold on; plot(DEM_Y_friston','-.')
% figure; plot(DEM_x(:,1:2),'-'); hold on; plot(DEM_Xest_friston','-.')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kalman filter with causes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% [kalmf, Lk, Pk] = kalmd(ss(A,[B,eye(nx)],C,zeros(ny,nx+1)),...
%                inv(P_brain_w),... +A*B*exp(0)*B'*A',...
%                inv(P_brain_z),sam_time);
% kalmfv_x = zeros(size(t,2),size(kalmf.C,2));
% for i = 2:size(t,2)
%     kalmfv_x(i,:)=(kalmf.A*kalmfv_x(i-1,:)' + kalmf.B*...
%                                     [real_cause(:,i); process_y(i,:)']);
% end
% 

%% All function definitions

function [model,brain] = generative_process(model,brain,if_dataset)

Tt  = 0:model.sam_time:model.t_end-model.sam_time;

brain.At = kron(eye(brain.p+1),model.A);
brain.Bt = kron(eye(brain.p+1,brain.d+1),model.B);
brain.Ct = kron(eye(brain.p+1),model.C);

T = toeplitz(zeros(1,brain.p+1),[0 1 zeros(1,brain.p-1)]);
if brain.p==0; T=0;end
brain.Da = kron(T,eye(size(model.A,2)));
T = toeplitz(zeros(1,brain.d+1),[0 1 zeros(1,brain.d-1)]);
if brain.d==0; T=0;end
brain.Dv = kron(T,eye(1));
brain.D_A = brain.Da-brain.At;

brain.nx = size(model.A,1);
brain.ny = size(model.C,1);

model.Pz = eye(brain.ny)/model.sigma_z^2;
model.Pw = eye(brain.nx)/model.sigma_w^2;
brain.Pz = eye(brain.ny)/brain.sigma_z^2;
brain.Pw = eye(brain.nx)/brain.sigma_w^2;

% sigma_brain_w = (diag([1.2341e-04,1.2341e-04,1.2341e-04]));
% sigma_brain_z = 7.5046e-06;
% P_brain_w = diag(1./(diag(sigma_brain_w).^2));
% P_brain_z = diag(1./(diag(sigma_brain_z).^2));

[brain.W0,brain.V0y,brain.V0v] = precision_matrix(brain.s,brain.Pw,...
                 brain.Pz,brain.sigma_v,brain.p,brain.d,brain.nx,brain.ny);
brain.V0 = blkdiag(brain.V0y,brain.V0v);

if if_dataset==0
    % generate noise
    [noise_w, noise_z] = make_noise(model.s,model.Pw,model.Pz,Tt,...
        model.sam_time,brain.nx,brain.ny,brain.nt);
    brain.noise_w = noise_w;
    brain.noise_z = noise_z;
    
    % s_est = estimate_smoothness(noise_z,Tt);
    
    % end
    % plot(s_realA,s_est,'x'); hold on; plot([0 1],[0 1])
    
    
    % generative process
    process = ss(model.A,[model.B   eye(brain.nx)  zeros(brain.nx,brain.ny)],...
        model.C,[zeros(brain.ny,1) zeros(brain.ny,brain.nx) eye(brain.ny)]);
    process = c2d(process,model.sam_time,'zoh');
    % [process_y,~,process_x] =lsim(process,[real_cause; noise_w; noise_z]...
    %                                ,t, zeros(2,1),'zoh');
    
    [process_x, process_y] = generate_data(model.A,model.B,model.C,noise_w,noise_z,...
        model.real_cause,model.t,model.sam_time,model.p);
    model.ideal_x = process_x;
    model.process_x = process_x;
    model.process_y = process_y;
end

end

function output = observer(model,brain,if_UIO,UIO_gamma,if_cause,...
    if_dataset,if_predict_y)

% Unknown Input Observer
if if_UIO
    [output.UIO_x_est, output.UIO_v_est] = UIO_estimator(...
        c2d(ss(model.A,model.B,model.C,0),model.sam_time,'zoh'),...
        model.ideal_x',model.process_y',brain.nt,model.real_cause,...
        if_dataset,UIO_gamma);
end

% DEM with unknown causes
[output.DEM_t,output.DEM_x] = D_step(model.A,model.B,model.C,...
    brain.Y_embed,brain.V0,brain.W0,...
    brain.At,brain.Bt,brain.Da,brain.Dv,brain.nx,brain.ny,brain.nt,...
                       brain.p,brain.d,model.t,model.sam_time,if_predict_y);

% Kalman Filter with unknown causes                  
if_causes = 0;                   
output.kalman_x = kalman_discrete(model.A,model.B,model.C,model.process_y,...
    brain.Pz,brain.Pw,model.sam_time,model.t,model.real_cause,if_causes);

if if_cause
    % D step with known causes
    output.DEMv_x = D_step_causes(model.A,brain.D_A,model.B,brain.Da,brain.Bt,brain.Ct,...
        brain.V0y,brain.W0,brain.Y_embed,model.real_cause,...
        model.t,model.sam_time,brain.nt,brain.ny,brain.p,brain.d);
    
    % Kalman Filter with causes
    output.kalmfv_x = KF_causes(model.A,model.B,model.C,brain.ny,model.sam_time,...
        brain.Pw,brain.Pz,model.real_cause,model.process_y,model.t,brain.nx);
end

end

function f = find_s(s_real,noise_z,Tt)
K  = toeplitz(exp(-Tt.^2/(2*s_real^2)));
K  = diag(1./sqrt(diag(K*K')))*K;

N = round(.25*size(noise_z,2));
corrupt_sig = noise_z*pinv(K);

corrupt_sig = corrupt_sig((end-N)/2+1:(end+N)/2);
f = sum((autocorr(corrupt_sig,'NumLags',30).^2)); % ,'NumLags',round(N/2)-1
% f = sum((autocorr(noise_z*pinv(K),'NumLags',50)>.2));

end

function [z_f, d_f] = UIO_estimator(sys,x_real,y_real,k,d,if_dataset,UIO_gamma)

A = sys.A; C = sys.C; D = sys.D;
B1 = zeros(size(A,1),1); B2 = sys.B; 

% A = [-.25 1; -.5 -.25];
% B = [1; 0];
% C = [.125 .1633; .125 .0676; .125 -.0676; .125 -.1633];

% A = [0.9323 0.0185; -0.0092 0.9138];
% B1 = [.1; 0];
% B2 = [.1; .1];
% C = [5 0];
% D = 0.1;
% H = [0 1];

% A = [0.9323 0.0194; -0.0097 1.01];
% B1 = [.1; 0];
% B2 = [.1; .1];
% C = [5 0];
% D = 0;

nx = size(A,1); ny = size(C,1); nu = size(B2,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Method 1, as per the paper, solve the LMI directly and tune gamma
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if if_dataset == 1
    P = sdpvar(nx,nx);
    Q = sdpvar(nu,nu);
    M = sdpvar(nu,nu);
    G = sdpvar(nx,ny);
    % gamma = 100;
    
    AA = [-P         zeros(nx,1)             (P*A-G*C)'     -(M*B2'*C'*C)'; ...
        zeros(1,nx)   -Q                      B2'*(P*A-G*C)' Q-B2'*(M*B2'*C'*C)'; ...
        (P*A-G*C)     (B2'*(P*A-G*C)')'       -P             zeros(nx,1); ...
        -(M*B2'*C'*C) Q'-(B2'*(M*B2'*C'*C)')' zeros(1,nx)    -Q];
    
    F = [P>=0, Q>=0, M>=0, AA<=0];
    optimize(F);
    P = value(P); Q = value(Q);
    M = value(M);
    G = value(G);
    L = inv(P)*G
    gamma = M*inv(Q)
    L = value(L)
    gamma = value(gamma)*1200000  % tune for best gamma; motor = gamma*1200000

else
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Method 2, as per paper, stable observer design through eigen values
    %          tune parameter UIO_gamma for best performance
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    L = sdpvar(nx,ny);
    gamma = sdpvar(nu,nu);
    AAA = [A-L*C (A-L*C)*B2; -gamma*B2'*C'*C eye(nu,nu)...
        - gamma*B2'*C'*C*B2];
    % optimize([sum(abs(eig(A-L*C)))<=1.75, ...
    %     sum(abs(eig(eye(ny)-C*B2*gamma*B2'*C')))<=.3, gamma>=.01, L>=0])
    optimize([sum(abs(eig(AAA)))<=size(AAA,1), gamma>=UIO_gamma])
    % optimize([max(abs(eig(AAA)))<=1, gamma>=UIO_gamma])
    
    L = value(L)
    gamma = value(gamma)   
    % value(abs(eig(AAA)))
end
% setlmis([])
% P = lmivar(1,[nx 1]);  Q = lmivar(1,[nu 1]);
% M = lmivar(2,[nu nu]); G = lmivar(2,[nx ny]);
% 
% lmiterm([1 1 1 P],-1,1); lmiterm([1 1 3 -P],A',1); lmiterm([1 1 3 -G],C',-1);
% lmiterm([1 1 4 -M],-C'*C*B2,1);
% lmiterm([1 2 2 Q],-1,1);
% lmiterm([1 2 3 -P],B2'*A',1); lmiterm([1 2 3 -G],B2'*C',-1);
% lmiterm([1 2 4 Q],1,1); lmiterm([1 2 4 -M],-B2'*C'*C*B2,1);
% lmiterm([1 3 3 P],-1,1);
% lmiterm([1 4 4 Q],-1,1);
% 
% lmiterm([-2 1 1 P],1,1);
% lmiterm([-3 1 1 Q],1,1);
% lmiterm([-4 1 1 M],1,1);  
% 
% LMISYS = getlmis;
% options = [0,0,1000,0,0];
% [tmin, solution] = feasp(LMISYS);%,options,-1);
% P = dec2mat(LMISYS,solution,P);
% Q = dec2mat(LMISYS,solution,Q);
% M = dec2mat(LMISYS,solution,M);
% G = dec2mat(LMISYS,solution,G);
% L = inv(P)*G;
% gamma = M*inv(Q);

% L = [.1886; .0969]; gamma = .9583;

% friston's system
% L = .4*L
% gamma = 15*gamma

% spring damper system
% L = .45*L;
% gamma = 4050*gamma;

% AAA = [A-L*C (A-L*C)*B2; -gamma*B2'*C'*C eye(size(gamma,1),size(B2,2))...
%                                     - gamma*B2'*C'*C*B2];
% abs(eig(AAA))
% fprintf('Stable observer? %d\n',sum(abs(eig(AAA))<1) == size(AAA,1));

% k = 500;
% d = zeros(1,k); 
u = zeros(1,k);
% d(1,125:320) = exp(-.0025*((125:320)-250).^2);%sin((125:320)/20);
% u(1,1:k) = 0;%sin((1:k)/20);
% x_real = zeros(nx,k);
% y_real = zeros(ny,k);
z_real = x_real;
% for i = 1:k
% %    x_real(:,i+1) = A*x_real(:,i) + B1*u(:,i) + B2*d(:,i);
% %    y_real(:,i) = C*x_real(:,i) + D*u(:,i);
%    z_real(:,i) = H*x_real(:,i); 
% end

d_f = zeros(nu,k);
x_f = zeros(nx,k);
y_f = zeros(ny,k);
z_f = zeros(nx,k);

d_f0 = .01;
d_f(:,1) = d_f0 + gamma*B2'*C'*(y_real(:,1) - C*B2*d_f0); % x_f(1) = 0
x_f(:,2) = A*B2*d_f0 + B1*u(:,k) + L*(y_real(:,1) - C*B2*d_f0);

for i = 2:k-1
    y_f(:,i) = C*x_f(:,i) + D*u(:,i) + C*B2*d_f(:,i-1);
    d_f(:,i) = d_f(:,i-1) + gamma*B2'*C'*(y_real(:,i) - y_f(:,i));
    z_f(:,i) = x_f(:,i) + B2*d_f(:,i-1);
    x_f(:,i+1) = A*x_f(:,i) + B1*u(:,i) + A*B2*d_f(:,i-1) + ...
                                     L*(y_real(:,i) - y_f(:,i));
end
y_f(:,k) = C*x_f(:,k) + D*u(:,k) + C*B2*d_f(:,k-1);
d_f(:,k) = d_f(:,k-1) + gamma*B2'*C'*(y_real(:,k) - y_f(:,k));
z_f(:,k) = x_f(:,k) + B2*d_f(:,k-1);

d_f(:,1:k-1) = d_f(:,2:k);
z_f(:,1:k-1) = z_f(:,2:k);

% figure; plot(d'); hold on; plot(d_f')
% sum((d-d_f).^2)
end

function [t,real_cause,process_x,process_y,ideal_y,current,voltage,Ts,A,B,C] ...
    = DC_motor_data

load C:\Users\ajithanilmeera\Desktop\DEM\DC_motor_data\R1\gbn_data_new_maxon_nogear;
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

function x = kalman_discrete(A,B,C,process_y,P_noise_z,P_noise_w,...
        sam_time,t,real_cause,if_causes)

Bd = (expm(A*sam_time)-eye(size(A)))*pinv(A)*B;

nx = size(A,1);
nt = size(t,2);
x_prev = zeros(nx,1);
Q = inv(P_noise_w);
R = inv(P_noise_z);
x  = zeros(nx,nt);         % EKF estimate of the mean of states
Q  = Q + expm(A)*B*exp(0)*B'*expm(A)';  % EKF process noise variance.
P  = {pinv(full(C'*R*C))};       % EKF conditional covariance of states
% P ={zeros(nx)};
for i = 2:nt
    
    % PREDICTION STEP:
    %----------------------------------------------------------------------
    xPred    = expm(A*sam_time)*x_prev + if_causes*Bd*real_cause(i);     % zero causes
    Jx       = expm(A);
    PPred    = Q + Jx*P{i-1}*Jx';
%     PPred =eye(2);
    
    % CORRECTION STEP:
    %----------------------------------------------------------------------
    yPred    = C*xPred;
    Jy       = C;
    K        = PPred*Jy'*inv(R + Jy*PPred*Jy');
    x_prev   = xPred + K*(process_y(i,:)' - yPred);
    x(:,i)   = x_prev;
    P{i}     = PPred - K*Jy*PPred;
%      P{i} =eye(2);
%     plot(i,A*x(:,i) + C'*P_noise_z*(process_y(i,:)' - C*x(:,i)),'r.'); hold on;
%     P{i}
    % report
    %----------------------------------------------------------------------
%     fprintf('EKF: time-step = %i : %i\n',i,nt);
    
end

% figure; plot(x')

end

function kalmfv_x = KF_causes(A,B,C,ny,sam_time,P_brain_w,P_brain_z,...
    real_cause,process_y,t,nx)

%%% General Kalman filter with causes
disc_sys = c2d(ss(A,B,C,zeros(ny,1)),sam_time);
[~,~,G1] = dare(disc_sys.A',disc_sys.C',inv(P_brain_w),inv(P_brain_z));

Akal = (disc_sys.A-G1'*disc_sys.C);
Bkal = [disc_sys.B G1'];
ukal = [real_cause; process_y'];

M3 = ss(Akal,Bkal,disc_sys.C,0,sam_time);
[~,~,kalmfv_x] = lsim(M3,ukal,t,zeros(nx,1));
kalmfv_x = kalmfv_x';

%%% Friston's Kalman Filter with causes
% if_causes = 1;                   
% kalmfv_x = kalman_discrete(A,B,C,process_y,P_brain_z,P_brain_w,...
%                            sam_time,t,real_cause,if_causes);

end

function [noise_w, noise_z] = make_noise(s_real,P_noise_w,P_noise_z,...
                                         Tt,sam_time,nx,ny,nt)

K  = toeplitz(exp(-Tt.^2/(2*s_real^2)));
K  = diag(1./sqrt(diag(K*K')))*K;
% rng(12);
noise_z = sqrtm(inv(P_noise_z))*randn(ny,nt)*K;
noise_w = sqrtm(inv(P_noise_w))*randn(nx,nt)*K*sam_time;

end

function s_est = estimate_smoothness(noise_z,Tt)

% s_incr = .005;
% corr_s = zeros(1,round(1/s_incr));
% for i = s_incr:s_incr:1
% corr_s(1,round(i/s_incr)) = find_s(i,noise_z,Tt);
% end
% figure(1); plot(s_incr:s_incr:1,corr_s,'r-');
% [~,min_i] = min(corr_s); 
% s_est = min_i*s_incr

% options = optimset('Display','iter');
func = @(k) find_s(k,noise_z,Tt);
% [s_est, ~] = fminbnd(func, 0,1)

% options = optimoptions('ga','PopulationSize',8,...
%     'MaxGenerations',15,'CrossoverFraction',.8);
% s_est = ga(func,1,[],[],[],[],0.01,1,[],[],options)

% options = optimoptions('simulannealbnd','Display','iter',...
%     'MaxFunctionEvaluations',100,'HybridFcn',@fmincon);
% s_est = simulannealbnd(func,50,0.01,1,options)

options = optimoptions('particleswarm','SwarmSize',50,...
            'HybridFcn',@fmincon,'MaxIterations',4,'Display','iter');
s_est = particleswarm(func,1,0.01,1,options)

% K  = toeplitz(exp(-Tt.^2/(2*s_est^2)));
% K  = diag(1./sqrt(diag(K*K')))*K;
% autocorr(noise_z*pinv(K),'NumLags',50)

end

function [W0,V0y,V0v] = precision_matrix(s_brain,P_brain_w,...
                                         P_brain_z,sigma_v,p,d,nx,ny) 

if p>d, pp = p; else pp = d; end

% Create the smoothness matrix
k          = 0:pp;
x          = sqrt(2)*s_brain;
r(1 + 2*k) = cumprod(1 - 2*k)./(x.^(2*k));
  
Cov     = [];
for i = 1:pp+1;
    Cov = [Cov; r([1:pp+1] + i - 1)];
    r = -r;
end

Cov_inv = inv(Cov);

W0 = kron(Cov_inv(1:p+1,1:p+1),P_brain_w);
V0y = kron(Cov_inv(1:p+1,1:p+1),P_brain_z);
V0v = kron(Cov_inv(1:d+1,1:d+1),eye(1)/sigma_v^2);

end

function Y_embed = generalized_process(process_y,t,sam_time,ny,p,d)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Embedding higher orders in the process output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nt = size(t,2);
E_y = [  1   0       0      0        0     0        0;
        [1 -1 0 0 0 0 0]/sam_time;
        [1 -2 1 0 0 0 0]/sam_time^2;
        [0 0 0 0 0 0 0];
        [0 0 0 0 0 0 0];
        [0 0 0 0 0 0 0];
        [0 0 0 0 0 0 0]]';

Y_embed = zeros(ny*(p+1)+(d+1),nt);
for i = 1:nt
    if i>p+1 && i<nt-p-1   % Use this section for low sampling time
        Y_embed(:,i) = [embed_Y(process_y',p+1,t(i),sam_time); -.5*ones(d+1,1)];%zeros(d+1,1)];
    else
        Y_embed(1:ny,i) = process_y(i,:)';
    end 
%     Y_embed(1:ny,i) = process_y(i,:)'; % No generalized output
%     Y_embed(:,i) = [embed_Y(process_y',p+1,t(i),sam_time); zeros(d+1,1)];
end
% figure; plot(Y_embed')

% for i = 1:nt
%    if(i > p) 
%        Y_embed(:,i) = [reshape(process_y(i:-1:i-p,:)'*E_y,[],1); zeros(d+1,1)];
%    else 
%        Y_embed(:,i) = [reshape([process_y(i:-1:1,:)' ...
%            repmat(process_y(1,:)',1,p+1-i)]*E_y,[],1); zeros(d+1,1)];
% %        Y_embed(:,i) = zeros((p+1)*4+(d+1),1);
% 
%    end 
% end

end

function [DEM_t,DEM_x] = D_step(A,B,C,Y_embed,V0,W0,At,Bt,Da,Dv,...
                                nx,ny,nt,p,d,t,sam_time,if_predict_y)
                          
T = toeplitz(zeros(1,p+1),[0 1 zeros(1,p-1)]);
if p==0; T=0;end
Dy = kron(T,eye(ny));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamic Expectation Maximization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k1 = [kron(eye(p+1),C') zeros(nx*(p+1),d+1)];
k2 = (At-Da)';
k3 = [zeros(d+1,ny*(p+1)) -eye(d+1)];
k4 = kron(eye(p+1,d+1),B)';

D_A = Da-At;

% Ct = kron(eye(p+1),C);
% kkk = sdpvar(1,1);
% kkk1 = sdpvar(1,1);
% AAA = [Da-kkk*D_A'*W0*D_A-kkk*Ct'*V0(1:ny*(p+1),1:ny*(p+1))*Ct kkk*D_A'*W0*Bt;...
%        kkk1*Bt'*W0*D_A                  Dv-kkk1*Bt'*W0*Bt-kkk1*V0(ny*(p+1)+1:end,ny*(p+1)+1:end)];
% 
% optimize([eig(AAA)<=0,kkk>=1, kkk1>=1])
% kkk = value(kkk)
% kkk1 = value(kkk1)
% 
% AAA1 = [(-eye(nx*(p+1))+kkk*D_A'*W0)*D_A Bt-kkk*D_A'*W0*Bt;...
%          -kkk1*Bt'*W0*D_A kkk1*(Bt'*W0*Bt+V0(ny*(p+1)+1:end,ny*(p+1)+1:end))];
% [~,dddd] = chol(AAA1)     

A1 = (k1*V0*blkdiag(kron(eye(p+1),-C),eye(d+1))+ ...
      k2*W0*[Da-At -Bt]) + [Da zeros((p+1)*nx,d+1)];
A2 = ((k3*V0*blkdiag(kron(eye(p+1),-C),eye(d+1))+ ...
      k4*W0*[Da-At -Bt])+ [zeros(d+1,(p+1)*nx) Dv]);
A3 = zeros(ny*(p+1));
A4 = Dv;

B1 = k1*V0;
B2 = k3*V0;
B3 = [Dy zeros(ny*(p+1),d+1)];
B4 = zeros(d+1,size(B1,2));

% Ad = expm([A1; A2]*sam_time);
% Bd = [A1; A2]\(Ad-eye(size(Ad)))*[B1;B2];
% state_sp = ss([A1;zeros(size(A2))], [B1;zeros(size(B2))], zeros(1,17), zeros(1,31));
if if_predict_y
    state_sp = ss(blkdiag([A1;A2],A3,A4), [B1;B2;B3;B4], ...
        zeros(1,(nx+ny)*(p+1)+(d+1)*2), zeros(1,ny*(p+1)+d+1));
else
    state_sp = ss([A1;A2], [B1;B2], ...
        zeros(1,(nx)*(p+1)+d+1), zeros(1,ny*(p+1)+d+1));
end
state_sp = c2d(state_sp,sam_time,'zoh');
% load('C:\Users\LocalAdmin\Desktop\DEM_test\D_step_Y_generalised.mat');
% [~,DEM_t,DEM_x] = lsim(state_sp,Y_embed,t,zeros(17,1),'zoh');
% figure; plot(DEM_x(:,[1:2 15])); hold on; plot(process_x);

% fprintf('\nStable?? %d\n',isstable(state_sp));

DEM_xx = zeros(nt,size(state_sp.C,2));
Vuu = -[k1 k2; k3 k4]*blkdiag(V0,W0)*[k1 k2; k3 k4]';
Vuy = [k1 k2; k3 k4]*blkdiag(V0,W0)*...
                [kron(eye(p+1),eye(ny)); zeros(nx*(p+1)+d+1,ny*(p+1))];
Vuv = [k1 k2; k3 k4]*blkdiag(V0,W0)*...
                [zeros(ny*(p+1),d+1); -eye(d+1); zeros(size(W0,1),d+1)];

dfdu = [Vuu Vuy Vuv; ...
        zeros(size(Dy,1),size([Vuu Vuy Vuv],2));...
        zeros(size(Dv,1),size([Vuu Vuy Vuv],2))]  + blkdiag(Da,Dv,Dy,Dv);

%     figure; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% delete this line
for i = 2:nt
    if if_predict_y
        q = [DEM_xx(i-1,1:nx*(p+1)+d+1) Y_embed(1:ny*(p+1)+d+1,i-1)'];
        f = blkdiag([A1; A2],A3,A4)*q' + [B1;B2;B3;B4]*Y_embed(:,i-1);
        DEM_xx(i,:) = q + ((expm(dfdu*sam_time)-eye(size(f,1)))*pinv(dfdu)*f)';
    else
        DEM_xx(i,:)=(state_sp.A*DEM_xx(i-1,:)' + state_sp.B*Y_embed(:,i))';
%         Da*DEM_xx(i,1:2)' ...
%             plot(i, D_A'*W0*(D_A*DEM_xx(i,1:nx)'),'r.'); hold on;
%             plot(i,Ct'*V0(1:ny,1:ny)*(Y_embed(1:ny,i)-Ct*DEM_xx(i,1:nx)'),'b.'); hold on;

%         plot(i,A'*W0*A*DEM_xx(i,1:2)','r.'); hold on;
%         plot(i,Ct'*V0(1:4,1:4)*(Y_embed(1:4,i)-Ct*DEM_xx(i,1:2)'),'b.'); hold on;
    
%         plot(i,pinv(A'*W0*A+C'*V0(1:ny,1:ny)*C)*C'*V0(1:ny,1:ny)*...
%             Y_embed(1:ny,i),'r.'); hold on;
%         plot(i,pinv(C)*Y_embed(1:ny,i),'b.'); hold on;
%         plot(i,pinv(-A  + care(A',C',inv(W0),inv(V0(1:ny,1:ny)))*C'*...
%             V0(1:ny,1:ny)*C)*care(A',C',inv(W0),inv(V0(1:ny,1:ny)))*C'*...
%             V0(1:ny,1:ny)*Y_embed(1:ny,i),'b.'); hold on;
%         
%         plot(i,(A*DEM_xx(i,1:nx)' + inv(W0)*C'*V0(1:ny,1:ny)*...
%             (Y_embed(1:ny,i)-Ct*DEM_xx(i,1:nx)')),'b.'); hold on;

%         -W0*inv(A')
    end
end
% plot(1:320,DEM_xx(:,1:nx)','r'); hold on;

% Pp = care(A',C',inv(W0),V0(1:ny,1:ny)); 
% yyy = lsim(ss(A-Pp*C'*V0(1:ny,1:ny)*C, Pp*C'*V0(1:ny,1:ny),eye(nx),0),...
%     Y_embed(1:ny,:),t);
% yyy1 = lsim(ss(-Pp*A'*W0*A-Pp*C'*V0(1:ny,1:ny)*C, Pp*C'*V0(1:ny,1:ny),...
%     eye(nx),0),Y_embed(1:ny,:),t);
% plot(yyy,'k'); hold on; plot(yyy1,'r'); hold on;

% kk = 1;
% for i=1:1:6
%    W0_ = (exp(-8)*eye(nx)); V0_ = exp(2*i)*eye(ny);
%    eig_dem(:,kk) = eig(-Pp*A'*W0_*A - Pp*C'*V0_*C) ;
%    Pp_ = care(A',C',inv(W0_),V0_);
%    eig_kal(:,kk) = eig(A - Pp*C'*V0_*C);
%    plot(real(eig_dem(1,kk)),imag(eig_dem(1,kk)),'ro'); hold on;   
%    plot(real(eig_dem(2,kk)),imag(eig_dem(2,kk)),'ro'); hold on;   
%    plot(real(eig_kal(1,kk)),imag(eig_kal(1,kk)),'bo'); hold on;   
%    plot(real(eig_kal(2,kk)),imag(eig_kal(2,kk)),'bo'); hold on;   
%    
%    kk= kk +1;
% end


DEM_x = DEM_xx;
DEM_t = t;

end

function DEMv_x = D_step_causes(A,D_A,B,Da,Bt,Ct,V0y,W0,Y_embed,real_cause,...
    t,sam_time,nt,ny,p_brain,d_brain)

state_sp_v = ss(Da - Ct'*V0y*Ct - D_A'*W0*D_A, ...
                [Ct'*V0y D_A'*W0*Bt], zeros(1,size(Da,2)), ...
                zeros(1,(p_brain+1)*ny+(d_brain+1)*size(B,2)));
state_sp_v = c2d(state_sp_v,sam_time,'zoh');

% Embed the known causes
V_embed = zeros(d_brain+1,nt);
% V_embed(1,:) = real_cause;
for i = 1:nt
    V_embed(:,i) = embed_Y(real_cause,d_brain+1,t(i),sam_time);
end
Y_embed(end-d_brain:end,:) = V_embed;

% Perform state estimation
DEMv_x = zeros(nt,size(state_sp_v.C,2));
% figure; 
for i = 2:nt
    DEMv_x(i,:)=(state_sp_v.A*DEMv_x(i-1,:)' + state_sp_v.B*Y_embed(:,i))';
%     plot(i,A'*W0*(A*DEMv_x(i,1:2)'+B*real_cause(i)),'r.'); hold on;
%     plot(i,Ct'*V0y*(Y_embed(1:4,i)-Ct*DEMv_x(i,1:2)'),'b.'); hold on;
end

end

function print_results(SSE,j,k,if_UIO,if_cause)

if if_UIO == 1
    fprintf('DEM_x, UIO_x, KF_x = ');
    disp(vpa([SSE.DEM.x(j,k) SSE.UIO.x(j,k) SSE.kalman.x(j,k)],4))
    if if_cause == 1
        fprintf('DEMv_x, KFv_x = ');
        disp(vpa([SSE.DEMv.x(j,k) SSE.kalmanv.x(j,k)],4))
    end
    fprintf('DEM_v, UIO_v = ');
    disp(vpa([SSE.DEM.v(j,k) SSE.UIO.v(j,k)],4));
else
    fprintf('DEM_x, KF_x, DEM_v = ');
    disp(vpa([SSE.DEM.x(j,k) SSE.kalman.x(j,k) SSE.DEM.v(j,k)],4))
    if if_cause == 1
        fprintf('DEMv_x, KFv_x = ');
        disp(vpa([SSE.DEMv.x(j,k) SSE.kalmanv.x(j,k)],4))
    end
    
end
end

function plot_results(output,model,brain,if_UIO,if_cause)
%% Plot the results

%
color = [ 0         0.4470    0.7410;    0.8500    0.3250    0.0980;
          0.9290    0.6940    0.1250;    0.4940    0.1840    0.5560;
          0.4660    0.6740    0.1880;    0.3010    0.7450    0.9330;
          0.6350    0.0780    0.1840];

% plot DEM with and without causes
if if_cause == 1
    figure;
    fig1 = plot(output.DEM_t,output.DEMv_x(:,1:brain.nx),'-.','Color',color(1,:)); hold on;
    fig2 = plot(output.DEM_t,output.kalmfv_x,'--','Color',color(2,:)); hold on;
    fig3 = plot(output.DEM_t,model.ideal_x,'-','Color',color(4,:));
    legend([fig1(1),fig2(1),fig3(1)],...
        {'DEMv states','KFv states','Ideal states'});
    xlabel time(s);
end

% plot state estimates and true states
figure; subplot(2,1,1);
fig1 = plot(model.t,model.ideal_x,'-','Color',color(1,:)); hold on;
fig2 = plot(output.DEM_t,output.DEM_x(:,1:brain.nx),'--','Color',color(2,:)); hold on;
fig3 = plot(model.t,output.kalman_x','-.','Color',color(4,:)); hold on;
if if_UIO == 1
    fig4 = plot(model.t,output.UIO_x_est,'-.','Color',color(5,:)); hold on;
    legend([fig1(1),fig2(1),fig3(1),fig4(1)],{'Ideal states','DEM states',...
        'Kalman estimate','UIO estimate'});    
else
    legend([fig1(1),fig2(1),fig3(1)],{'Ideal states','DEM states','Kalman estimate'});
end

% plot causal state estimate and true cause
subplot(2,1,2)
plot(output.DEM_t,output.DEM_x(:,brain.nx*(brain.p+1)+1),'--'); hold on;
plot(model.t,model.real_cause','-','LineWidth',1); hold on;
if if_UIO == 1
    plot(model.t,output.UIO_v_est,'-.'); hold on;
    legend('DEM cause estimate','Ideal causal state','UIO cause estimate');
else
    legend('DEM cause estimate','Ideal causal state');    
end

% plot kalman state estimates and true states
% figure; 
% fig1 = plot(output.DEM_t,output.DEM_x(:,1:brain.nx),'--','Color',color(1,:)); hold on;
% fig2 = plot(model.t,output.kalman_x','-.','Color',color(2,:)); hold on;
% fig3 = plot(model.t,model.ideal_x,'-','Color',color(4,:));
% legend([fig1(1),fig2(1),fig3(1)],...
%     {'DEM states','Kalman states','Ideal states'});
% xlabel time(s)
end

function [process_x, process_y] = generate_data(A,B,C,noise_w,noise_z,...
                                                real_cause,t,sam_time,p)
if_GC = 0; % If generalized coordinates should be used in generative process

nx = size(A,1);
ny = size(C,1);
nt = size(t,2);

if if_GC == 0
    ideal_sp = c2d(ss(A,B,C,zeros(ny,1)),sam_time,'zoh');
end

process_x = zeros(nx,nt)';
process_y = zeros(ny,nt)';

U.x = zeros(nx,p+1);
U.v = zeros(ny+1,p+1);
process_y(1,:) = noise_z(:,1);
if if_GC == 1
    for i=1:nt-1
        % generalized noises and causes
        d_noise_w = embed_Y(noise_w,p+1,t(i),sam_time);
        d_noise_z = embed_Y([noise_z; real_cause],p+1,t(i),sam_time);
        d_noise_w = reshape(d_noise_w,nx,[]);
        d_noise_z = reshape(d_noise_z,ny+1,[]);
        
        pU.x(:,1)  = U.x(:,1);
        pU.v(:,1)  = [C*U.x(:,1); 0] + [noise_z(:,i); real_cause(1,i)];
        pU.x(:,2)  = A*U.x(:,1) + [zeros(size(C')) B]*U.v(:,1) + noise_w(:,i);
        for j = 2:p
            pU.v(:,j) = [C*pU.x(:,j); 0] + d_noise_z(:,j);
            pU.x(:,j+1) = [zeros(size(C')) B]*pU.v(:,j) + A*pU.x(:,j) + ...
                d_noise_w(:,j);
        end
        pU.v(:,p+1) = [C*pU.x(:,p+1); 0] + d_noise_z(:,p+1);
        
        Dv_j = kron(diag(ones(1,p),1),eye(ny+1));
        Dx_j = kron(diag(ones(1,p),1),eye(nx));
        
        dgdv = zeros((p+1)*(ny+1));
        dgdx = kron(diag(ones(1,p),1),[C; zeros(1,nx)]);
        dfdv = kron(eye(p+1),[zeros(size(C')) B]);
        dfdx = kron(eye(p+1),A);
        dfdw = kron(eye(p+1),eye(nx));
        J = [dgdv dgdx Dv_j zeros(size(Dv_j,1),size(Dx_j,2));...
            dfdv dfdx zeros((p+1)*nx,size(Dv_j,2)) dfdw;...
            zeros((p+1)*[(ny+1),(ny+1+nx)]) Dv_j zeros(size(Dv_j,1),size(Dx_j,2));...
            zeros((p+1)*[nx,(ny+1+nx)]) zeros(size(Dx_j,1),size(Dv_j,2)) Dx_j];
        qU = [reshape(pU.v,[],1); reshape(pU.x,[],1); ...
            reshape(d_noise_z,[],1); reshape(d_noise_w,[],1)];
        D_j = blkdiag(Dv_j,Dx_j,Dv_j,Dx_j);
        qU = qU + (expm(J*sam_time) - eye(size(J)))*pinv(J)*D_j*qU;
        
        U.v = reshape(qU(1:(p+1)*(ny+1),1),ny+1,p+1);
        U.x = reshape(qU((p+1)*(ny+1)+1:(p+1)*(ny+1+nx),1),...
            nx,p+1);
        process_y(i+1,:) = U.v(1:ny,1);
        process_x(i+1,:) = U.x(:,1);
    end
else
    % Generative process without generalized motion
    for i=2:nt
        process_x(i,:) = (ideal_sp.A*process_x(i-1,:)' + ...
            ideal_sp.B*real_cause(1,i-1)+ noise_w(:,i-1))';
        process_y(i-1,:) = (ideal_sp.C*process_x(i-1,:)' + ...
            ideal_sp.D*real_cause(1,i-1)+ noise_z(:,i-1))';
    end
end
if if_GC == 0
    process_y(end,:) = (ideal_sp.C*process_x(end,:)' + ...
        ideal_sp.D*real_cause(1,end)+ noise_z(:,end))';
end

end

function dY = embed_Y(Y,n,t,dt)

    [~, N]  = size(Y);
    T = zeros(n);
    
    s      = round((t)/dt);
    k      = (1:n)  + fix(s - (n + 1)/2);
    x      = s - min(k) + 1;
    y_pad = k<x-1 | k> N-(x-1);% Extra step: truncate innacurate derivative at edges
    i      = k < 1;
    k      = k.*~i + i;
    i      = k > N;
    k      = k.*~i + i*N;


    % Inverse embedding operator (T): cf, Taylor expansion Y(t) <- T*y[:]
    %----------------------------------------------------------------------
    for i = 1:n
        for j = 1:n
            T(i,j) = ((i - x)*dt)^(j - 1)/prod(1:(j - 1));
        end
    end

    % embedding operator: y[:] <- E*Y(t)
    %----------------------------------------------------------------------
    E     = inv(T);
   
    dY      = Y(:,k)*E';
    dY(:,end-sum(y_pad)+1:end) = zeros(size(Y,1),sum(y_pad)); % truncate inaccurate derivatives
    dY = reshape(dY,[],1);
end
