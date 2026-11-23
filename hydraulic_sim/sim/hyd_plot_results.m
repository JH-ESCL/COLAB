function hyd_plot_results(R)
%HYD_PLOT_RESULTS  압력 / 유량 / 온도 / 기타 결과 플롯
%   R.t, R.mon.<LRU> (각 열 = 해당 lru_<type>_f 의 mon 정의)
psi = 6894.757; Patm = 101325; lpm = 1e-3/60;
t = R.t; M = R.mon;
g = @(P) (P - Patm)/psi;   % abs Pa -> psig

figure('Name', 'Pressure', 'Color', 'w');
plot(t, g(M.PMAN(:,1)), t, g(M.EDP(:,1)), t, g(M.ACMP(:,1)), t, g(M.ACC(:,1)), ...
     t, g(M.ACT(:,4)), t, g(M.RTF(:,1)), t, g(M.EDP(:,6)), 'LineWidth', 1.1);
grid on; xlabel('time [s]'); ylabel('pressure [psig]');
legend('PMAN','EDP out','ACMP out','ACC','ACT supply','RTF out','EDP case', 'Location','best');
title('System pressure');

figure('Name', 'Suction', 'Color', 'w');
plot(t, g(M.RSV(:,1)), t, g(M.EDP(:,4)), t, g(M.ACMP(:,4)), 'LineWidth', 1.1); grid on;
yyaxis right; plot(t, M.EDP(:,13), '--'); ylabel('EDP fill factor f_{cav}');
yyaxis left; xlabel('time [s]'); ylabel('pressure [psig]');
legend('RSV','EDP inlet','ACMP inlet','EDP f_{cav}', 'Location','best'); title('Suction side');

figure('Name', 'Flow', 'Color', 'w');
plot(t, M.EDP(:,2)/lpm, t, M.ACMP(:,2)/lpm, t, M.EDP(:,7)/lpm, t, M.ACMP(:,7)/lpm, ...
     t, M.ACT(:,8)/lpm, t, M.RV(:,2)/lpm, t, M.RSV(:,2)/lpm, 'LineWidth', 1.1);
grid on; xlabel('time [s]'); ylabel('flow [L/min]');
legend('EDP delivery','ACMP delivery','EDP case drain','ACMP case drain','ACT supply', ...
       'RV relief','RSV return (total)', 'Location','best');
title('Flow');

figure('Name', 'Temperature', 'Color', 'w');
plot(t, M.RSV(:,3), t, M.EDP(:,8), t, M.ACMP(:,8), t, M.CDF(:,3), t, M.HX(:,3), ...
     t, M.PMAN(:,3), t, M.ACT(:,10), t, M.RTF(:,3), 'LineWidth', 1.1);
grid on; xlabel('time [s]'); ylabel('temperature [degC]');
legend('RSV','EDP case','ACMP case','CDF out (HX in)','HX out','PMAN','ACT','RTF out', ...
       'Location','best');
title('Fluid temperature');

figure('Name', 'Misc', 'Color', 'w');
subplot(3,1,1); plot(t, 1e3*M.ACT(:,2), t, 1e3*M.ACT(:,1)); grid on;
ylabel('x [mm]'); legend('cmd','piston'); title('Servo actuator');
subplot(3,1,2); plot(t, M.EDP(:,9), t, M.ACMP(:,9)); grid on; ylabel('\epsilon [-]');
legend('EDP yoke','ACMP yoke');
subplot(3,1,3); plot(t, M.RSV(:,7), t, M.HX(:,4)/1e3); grid on; xlabel('time [s]');
legend('RSV level [%]','HX heat rejection [kW]');
end
