function f = find_s(s_real,noise_z,Tt)
K  = toeplitz(exp(-Tt.^2/(2*s_real^2)));
K  = diag(1./sqrt(diag(K*K')))*K;

N = round(.25*size(noise_z,2));
corrupt_sig = noise_z*pinv(K);

corrupt_sig = corrupt_sig((end-N)/2+1:(end+N)/2);
f = sum((autocorr(corrupt_sig,'NumLags',30).^2)); % ,'NumLags',round(N/2)-1
% f = sum((autocorr(noise_z*pinv(K),'NumLags',50)>.2));

end