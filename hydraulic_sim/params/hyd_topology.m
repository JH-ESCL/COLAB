function topo = hyd_topology()
%HYD_TOPOLOGY  유압 시스템 LRU 구성 및 배관 연결 정의 (Simulink 빌더 / ODE 레퍼런스 공용)
%
%  RSV ─┬─ SOV ── SUPL_E ── EDP ──(P)── PFM_E ── CKV_E ─┐
%       │                    └──(CD)──────────┐          ├─ PMAN ─┬─ ACC
%       └─ SUPL_M ─────────── ACMP ─(P)── PFM_M ── CKV_M ┘        ├─ RV ────────────┐
%                              └──(CD)───────┤                    └─ PLINE ─ ACT ─ RLINE ─┤
%                                            CDF ── HX ── RSV(CD)                   RTF ── RSV(RET)
%
%  blocks : {이름, 타입(lru_<type>_f/g), Nin(입구 수), Nout(출구 연결 수), extId(시나리오 ID)}
%  conns  : {상류, 상류 출구번호, 하류, 하류 입구번호}
%           신호 규약:  상류 -> 하류 : fwd = [P; T]
%                       하류 -> 상류 : bwd = [Q; T]
blocks = {
%  name      type      Nin Nout extId
  'RSV'     'rsv'      2   2   0
  'SOV'     'sov'      1   1   1
  'SUPL_E'  'line'     1   1   0
  'SUPL_M'  'line'     1   1   0
  'EDP'     'edp'      1   2   2
  'ACMP'    'acmp'     1   2   3
  'PFM_E'   'filter'   1   1   0
  'PFM_M'   'filter'   1   1   0
  'CKV_E'   'ckv'      1   1   0
  'CKV_M'   'ckv'      1   1   0
  'PMAN'    'man'      2   3   0
  'ACC'     'acc'      1   0   0
  'RV'      'rv'       1   1   0
  'PLINE'   'line'     1   1   0
  'ACT'     'act'      1   1   4
  'RLINE'   'line'     1   1   0
  'RTF'     'filter'   2   1   0
  'CDF'     'filter'   2   1   0
  'HX'      'hx'       1   1   5
};
conns = {
%  from     out  to        in
  'RSV'     1   'SOV'      1
  'RSV'     2   'SUPL_M'   1
  'SOV'     1   'SUPL_E'   1
  'SUPL_E'  1   'EDP'      1
  'SUPL_M'  1   'ACMP'     1
  'EDP'     1   'PFM_E'    1
  'EDP'     2   'CDF'      1
  'ACMP'    1   'PFM_M'    1
  'ACMP'    2   'CDF'      2
  'PFM_E'   1   'CKV_E'    1
  'PFM_M'   1   'CKV_M'    1
  'CKV_E'   1   'PMAN'     1
  'CKV_M'   1   'PMAN'     2
  'PMAN'    1   'ACC'      1
  'PMAN'    2   'RV'       1
  'PMAN'    3   'PLINE'    1
  'PLINE'   1   'ACT'      1
  'ACT'     1   'RLINE'    1
  'RLINE'   1   'RTF'      1
  'RV'      1   'RTF'      2
  'RTF'     1   'RSV'      1
  'CDF'     1   'HX'       1
  'HX'      1   'RSV'      2
};
topo.blocks = cell2struct(blocks, {'name','type','Nin','Nout','extId'}, 2);
topo.conns  = cell2struct(conns,  {'from','out','to','in'}, 2);
end
