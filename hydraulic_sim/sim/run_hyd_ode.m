%RUN_HYD_ODE  Simulink 없이 MATLAB ode15s 로 동일 모델을 해석 (레퍼런스 / 검증용)
hyd_init;
P = hyd_params();
S = hyd_sys_index(P);
opts = odeset('RelTol', 1e-4, 'AbsTol', S.atol, 'MaxStep', 0.05, 'MaxOrder', 2);
tic;
[t, X] = ode15s(@(t, x) hyd_sys_rhs(t, x, S), [0 P.tEnd], S.X0, opts);
fprintf('ode15s: %d steps, %.1f s\n', numel(t), toc);

% 모니터 신호 재구성
M = struct();
for n = 1:numel(t)
    [~, mon] = hyd_sys_rhs(t(n), X(n,:)', S);
    for k = 1:S.nb
        M.(S.names{k})(n, :) = mon{k}.';
    end
end
R.t = t; R.X = X; R.mon = M; R.S = S;
hyd_plot_results(R);
