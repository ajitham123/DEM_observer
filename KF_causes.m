function kalmfv_x = KF_causes(A,B,C,nv,ny,sam_time,P_brain_w,P_brain_z,...
    real_cause,process_y,t,nx)

%%% General Kalman filter with causes
disc_sys = c2d(ss(A,B,C,zeros(ny,nv)),sam_time);
[~,~,G1] = dare(disc_sys.A',disc_sys.C',inv(P_brain_w),inv(P_brain_z));

Akal = (disc_sys.A-G1'*disc_sys.C);
Bkal = [disc_sys.B G1'];
ukal = [real_cause; process_y'];

M3 = ss(Akal,Bkal,disc_sys.C,0,sam_time);
[~,~,kalmfv_x] = lsim(M3,ukal,t,zeros(nx,1));
kalmfv_x = kalmfv_x';
end