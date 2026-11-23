function [x0, atol] = hyd_lru_init(type, prm, P0, T0)
%HYD_LRU_INIT  LRU 타입별 상태 초기값 및 적분 절대오차(스케일) 벡터
%   상태 종류별 AbsTol :  Q 1e-7 m3/s,  P 1e2 Pa,  T 1e-3 K,  V 1e-7 m3,  개도 1e-5  (RelTol 1e-4 와 함께 사용)
aQ = 1e-7; aP = 1e2; aT = 1e-3; aV = 1e-7; aO = 1e-5;
switch type
    case 'line'
        x0 = [0; P0; T0];                 atol = [aQ; aP; aT];
    case {'sov'}
        x0 = [0; P0; T0; 1];              atol = [aQ; aP; aT; aO];
    case {'ckv','rv'}
        x0 = [0; P0; T0; 0];              atol = [aQ; aP; aT; aO];
    case 'hx'
        x0 = [0; P0; T0];                 atol = [aQ; aP; aT];
    case 'rsv'
        n  = prm.Nin;
        x0 = [zeros(n,1); prm.Vfl0; prm.Preg; T0];
        atol = [aQ*ones(n,1); aV; aP; aT];
    case 'man'
        n  = prm.Nin;
        x0 = [zeros(n,1); P0; T0];        atol = [aQ*ones(n,1); aP; aT];
    case 'filter'
        n  = prm.Nin;
        x0 = [zeros(n,1); P0; T0; 0; P0; T0];
        atol = [aQ*ones(n,1); aP; aT; aQ; aP; aT];
    case 'edp'
        x0 = [0; P0; T0; 1; P0; T0; P0; T0];
        atol = [aQ; aP; aT; aO; aP; aT; aP; aT];
    case 'acmp'
        x0 = [0; P0; T0; 1; P0; T0; P0; T0; 0];
        atol = [aQ; aP; aT; aO; aP; aT; aP; aT; 1e-4];
    case 'acc'
        x0 = [0; P0; T0];                 atol = [aQ; aP; aT];
    case 'act'
        x0 = [0; P0; P0; P0; P0; 0; 0; 0; T0];
        atol = [aQ; aP; aP; aP; aP; 1e-8; 1e-7; aO; aT];
    otherwise
        error('hyd_lru_init:type', 'Unknown LRU type "%s"', type);
end
end
