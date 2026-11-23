%RUN_HYD_SIM  Simulink 모델 생성(없으면) -> 시뮬레이션 -> 결과 플롯
hyd_init;
P = hyd_params();
mdl = 'hyd_system';
if ~exist([mdl '.slx'], 'file')
    build_hyd_system(mdl);
end
tic;
out = sim(mdl, 'StopTime', num2str(P.tEnd));
fprintf('Simulink: %.1f s\n', toc);

% SimulationOutput -> 공통 결과 구조체
R.mon = struct();
names = {P.topo.blocks.name};
for k = 1:numel(names)
    ts = out.get(['mon_' names{k}]);
    R.mon.(names{k}) = squeeze(ts.Data);
    if size(R.mon.(names{k}), 1) ~= numel(ts.Time)
        R.mon.(names{k}) = R.mon.(names{k}).';
    end
    R.t = ts.Time;
end
hyd_plot_results(R);
