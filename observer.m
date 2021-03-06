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
    brain.At,brain.Bt,brain.Da,brain.Dv,brain.nv,brain.nx,brain.ny,brain.nt,...
                       brain.p,brain.d,model.t,model.sam_time,if_predict_y);

% Kalman Filter with unknown causes                  
if_causes = 0;                   
output.kalman_x = kalman_discrete(model.A,model.B,model.C,model.process_y,...
    brain.Pz,brain.Pw,model.sam_time,model.t,model.real_cause,if_causes);

if if_cause
    % D step with known causes
    output.DEMv_x = D_step_causes(model.A,brain.D_A,model.B,brain.Da,brain.Bt,brain.Ct,...
        brain.V0y,brain.W0,brain.Y_embed,model.real_cause,...
        model.t,model.sam_time,brain.nt,brain.nv,brain.ny,brain.p,brain.d);
    
    % Kalman Filter with causes
    output.kalmfv_x = KF_causes(model.A,model.B,model.C,brain.nv,brain.ny,model.sam_time,...
        brain.Pw,brain.Pz,model.real_cause,model.process_y,model.t,brain.nx);
end

end