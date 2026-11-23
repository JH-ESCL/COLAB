function dT = hyd_energy(T, rho, cp, V, Qin, Tin, Qout, Tout, Qheat, UA, Tamb)
%HYD_ENERGY  집중 체적(lumped volume)의 에너지 방정식
%   rho*cp*V*dT/dt = rho*cp*[ sum(Qin+ (Tin-T)) + sum(Qout- (Tout-T)) ]
%                    + Qheat - UA*(T - Tamb)
%   Qin  : 상류에서 노드로 들어오는 유량 (+ 유입)   Tin  : 상류 온도
%   Qout : 노드에서 하류로 나가는 유량  (+ 유출)   Tout : 하류 온도 (역류시 사용)
%   Qheat: 노드에 투입되는 열 [W] (교축손실, 마찰 등)
%#codegen
H  = sum(max(Qin, 0).*(Tin - T)) + sum(max(-Qout, 0).*(Tout - T));
dT = (rho*cp*H + Qheat - UA*(T - Tamb)) / (rho*cp*max(V, 1e-9));
end
