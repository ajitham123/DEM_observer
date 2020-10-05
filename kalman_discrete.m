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
    xPred    = expm(A*sam_time)*x_prev + if_causes*Bd*real_cause(:,i);     % zero causes
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