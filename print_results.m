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