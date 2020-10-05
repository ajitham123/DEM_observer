function s_est = estimate_smoothness(noise_z,Tt)
func = @(k) find_s(k,noise_z,Tt);
options = optimoptions('particleswarm','SwarmSize',50,...
            'HybridFcn',@fmincon,'MaxIterations',4,'Display','iter');
s_est = particleswarm(func,1,0.01,1,options)
end