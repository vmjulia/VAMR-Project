function projected = project_point(X, Rt, K)
    x_cam = Rt * X;
    homo = K * x_cam(1:3) / x_cam(4);
    projected = homo(1:2) / homo(3);
end