function [noise_w, noise_z] = make_noise(s_real,P_noise_w,P_noise_z,...
                                         Tt,sam_time,nx,ny,nt)

K  = toeplitz(exp(-Tt.^2/(2*s_real^2)));
K  = diag(1./sqrt(diag(K*K')))*K;
% rng(12);
noise_z = sqrtm(inv(P_noise_z))*randn(ny,nt)*K;
noise_w = sqrtm(inv(P_noise_w))*randn(nx,nt)*K*sam_time;

end