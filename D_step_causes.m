function DEMv_x = D_step_causes(A,D_A,B,Da,Bt,Ct,V0y,W0,Y_embed,real_cause,...
    t,sam_time,nt,nv,ny,p_brain,d_brain)

state_sp_v = ss(Da - Ct'*V0y*Ct - D_A'*W0*D_A, ...
                [Ct'*V0y D_A'*W0*Bt], zeros(1,size(Da,2)), ...
                zeros(1,(p_brain+1)*ny+(d_brain+1)*nv));
state_sp_v = c2d(state_sp_v,sam_time,'zoh');

% Embed the known causes
V_embed = zeros(nv*(d_brain+1),nt);
% V_embed(1,:) = real_cause;
for i = 1:nt
    V_embed(:,i) = embed_Y(real_cause,d_brain+1,t(i),sam_time);
end
Y_embed(ny*(p_brain+1)+1:end,:) = V_embed;

% Perform state estimation
DEMv_x = zeros(nt,size(state_sp_v.C,2));
% figure; 
for i = 2:nt
    DEMv_x(i,:)=(state_sp_v.A*DEMv_x(i-1,:)' + state_sp_v.B*Y_embed(:,i))';
end

end