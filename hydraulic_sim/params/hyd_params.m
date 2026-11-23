function P = hyd_params()
%HYD_PARAMS  유압 시스템 / LRU 별 파라미터 (SI 단위, 압력 = 절대압 [Pa], 온도 [degC])
%   P.fl          : 작동유 물성
%   P.blk.<NAME>  : LRU 인스턴스 파라미터 (+ Nin, Nout, x0, atol 자동 추가)
%   P.topo        : HYD_TOPOLOGY
%   P.tEnd        : 시뮬레이션 종료 시간
psi  = 6894.757;
Patm = 101325;
lpm  = 1e-3/60;                % L/min -> m^3/s

%% 작동유 (MIL-PRF-83282 근사. Skydrol LD-4 사용시 rho0~990, nu0~11e-6, cp~1750)
fl.rho0   = 832;               % [kg/m3] @T0
fl.T0     = 40;                % [degC]
fl.alpha  = 7.5e-4;            % 체적 열팽창 [1/K]
fl.beta0  = 1.5e9;             % 액체 체적탄성계수 @T0, Patm [Pa]
fl.kbT    = 0.004;             % beta 온도 감소율 [1/K]
fl.kbP    = 10;                % dbeta/dP [-]
fl.nu0    = 14e-6;             % 동점도 @40degC [m2/s]
fl.lambda = log(14/3.5)/60;    % 3.5 cSt @100degC
fl.cp     = 2000;              % [J/kg/K]
fl.Patm   = Patm;
fl.xair   = 0.002;             % 혼입 공기 체적비 @Patm
fl.nair   = 1.4;
fl.Pmin   = 1e3;
P.fl = fl;

P.Tamb = 20;                   % 장착 구역 온도 [degC]
P.T0   = 25;                   % 초기 유체 온도
Preg   = Patm + 45*psi;        % 저장조 가압
P.P0   = Preg;                 % 초기 압력 (전 계통)
P.tEnd = 1200;

%% LRU 파라미터 ---------------------------------------------------------------
b = struct();

% RSV : 저장조
b.RSV = struct('Vtot',30e-3,'Vfl0',18e-3,'Abase',0.05,'nz',1,'Preg',Preg, ...
    'tau_reg',1.0,'ngas',1.0,'Lin',0.5,'Din',0.032,'Kin',1.0,'Vmin',1e-3,'UA',8);

% SOV : 공급 차단밸브 (+ 짧은 배관)
b.SOV = struct('L',0.3,'D',0.032,'CdA_max',4.5e-4,'CdA_leak',1e-10, ...
    'tau',0.05,'rate',1/1.0,'V',pi*0.032^2/4*0.3 + 0.1e-3,'UA',1);

% SUPL_E / SUPL_M : 흡입 라인
b.SUPL_E = mkline(4.0, 0.032, 1.5);
b.SUPL_M = mkline(2.5, 0.019, 1.5);
% PLINE : 압력 분배 라인,  RLINE : 리턴 라인
b.PLINE  = mkline(15, 0.016, 3);
b.RLINE  = mkline(15, 0.019, 3);

% EDP : 엔진 구동 펌프 (37 gpm @3750 rpm 급)
%  압력보상 루프 교차주파수 wc = (Qmax/(Pdead-Pfull)) / (V_out/beta) ~ 수천 rad/s
%  -> tau_c 는 1/wc 보다 충분히 작아야 함 (위상여유). Vout 은 토출 호스 체적 포함.
b.EDP = struct('Dmax',3.7e-5,'Pfull',Patm+2850*psi,'Pdead',Patm+3025*psi, ...
    'tau_c',0.5e-3,'Cleak',4e-12,'nu_ref',fl.nu0,'Pcav',50e3,'dPcav',30e3, ...
    'eta_m',0.92,'Tdrag',1.5,'phi_out',0.3,'Lin',0.1,'Din',0.032, ...
    'Vin',0.2e-3,'Vout',0.4e-3,'Vcase',0.5e-3,'UA',3);

% ACMP : 교류 모터 펌프 (10 gpm @11500 rpm 급, 400 Hz 4극)
b.ACMP = struct('Dmax',3.2e-6,'Pfull',Patm+2850*psi,'Pdead',Patm+3025*psi, ...
    'tau_c',0.5e-3,'Cleak',1.5e-12,'nu_ref',fl.nu0,'Pcav',50e3,'dPcav',30e3, ...
    'eta_m',0.90,'Tdrag',0.3,'phi_out',0.3,'Lin',0.1,'Din',0.019, ...
    'Vin',0.1e-3,'Vout',0.3e-3,'Vcase',0.2e-3,'UA',2, ...
    'w_sync',2*pi*12000/60,'Tmax',30,'smax',0.3,'J',3e-3,'b_m',1e-4,'eta_e',0.85);

% PFM : 압력 필터 모듈 (바이패스 없음, Kbp = 0)
b.PFM_E = mkfilter(0.1, 0.025, 0.14e6/(140*lpm), 0, 100*psi, 70*psi, 0.3e-3, 0.2e-3);
b.PFM_M = mkfilter(0.1, 0.016, 0.14e6/(37*lpm),  0, 100*psi, 70*psi, 0.15e-3, 0.1e-3);
% RTF : 리턴 필터 모듈 (바이패스 100 psid)
b.RTF   = mkfilter(0.3, 0.025, 0.10e6/(140*lpm), 5e-9, 100*psi, 70*psi, 0.6e-3, 0.4e-3);
% CDF : 케이스드레인 필터 (입구 = 케이스드레인 라인 1.5 m, 1/2")
b.CDF   = mkfilter(1.5, 0.010, 70e3/(8*lpm), 1e-9, 50*psi, 35*psi, 0.2e-3, 0.15e-3);

% CKV : 펌프 토출 체크밸브 (5 psid crack, 정격유량에서 +50 psid)
b.CKV_E = mkpoppet(0.05, 0.025, 5*psi, 50*psi/(140*lpm), 2.5e-4, 140*lpm, 0.01, 0.05e-3);
b.CKV_M = mkpoppet(0.05, 0.016, 5*psi, 50*psi/(37*lpm),  8e-5,   37*lpm, 0.01, 0.03e-3);

% PMAN : 압력 매니폴드
b.PMAN = struct('Lin',0.3,'Din',0.025,'Kin',1.0,'V',0.5e-3,'UA',3);

% ACC : 어큐뮬레이터 (1 L, 1500 psi 예압)
b.ACC = struct('Lp',0.3,'Dp',0.019,'CdA_port',1e-4,'V0',1.0e-3, ...
    'Ppre',Patm+1500*psi,'ngas',1.2,'Vdead',0.05e-3,'dP_s',0.1e6,'UA',2);

% RV : 시스템 릴리프 밸브 (3500 psid crack, 3650 psid @ 140 L/min)
b.RV = mkpoppet(0.05, 0.012, 3500*psi, 150*psi/(140*lpm), 1.2e-5, 140*lpm, 0.001, 0.05e-3);

% ACT : 서보 액추에이터
Ap = 3e-3; xmax = 0.05; Vdead = 0.05e-3;
b.ACT = struct('Lp',0.2,'Dp',0.012,'Vs',0.05e-3,'Vr',0.05e-3,'Ap',Ap,'xmax',xmax, ...
    'V0A',Vdead+Ap*xmax,'V0B',Vdead+Ap*xmax,'m',100,'b',3000,'Fc',200,'vc',0.005, ...
    'kaero',4e5,'kst',1e8,'cst',1e5,'Kx',150,'tau_v',0.004, ...
    'CdA_max',3.7e-6,'CdA_ul',3e-8,'Cip',1e-13,'UA',5);

% HX : 케이스드레인 연료 냉각 열교환기
b.HX = struct('R0',5e8,'nu_ref',fl.nu0,'K2',0,'I',fl.rho0*2/2e-4,'V',0.4e-3, ...
    'UA_hx',150,'UA_s',5,'cp_c',2000,'UA',0);

%% 토폴로지 정보 / 초기조건 / 공통 필드 추가
topo = hyd_topology();
for k = 1:numel(topo.blocks)
    bk  = topo.blocks(k);
    prm = b.(bk.name);
    prm.fl   = fl;
    prm.Tamb = P.Tamb;
    prm.Nin  = bk.Nin;
    prm.Nout = bk.Nout;
    [prm.x0, prm.atol] = hyd_lru_init(bk.type, prm, P.P0, P.T0);
    b.(bk.name) = prm;
end
P.blk  = b;
P.topo = topo;
end

% -------------------------------------------------------------------------
function s = mkline(L, D, K)
A = pi*D^2/4;
s = struct('L',L,'D',D,'K',K,'V',A*L,'UA',10*pi*D*L);  % 자연대류 10 W/m2K
end

function s = mkfilter(Lin, Din, R0, Kbp, dp_crack, dp_ind, Vh, Vo)
s = struct('Lin',Lin,'Din',Din,'R0',R0,'nu_ref',14e-6,'kclog',9,'clog',0, ...
    'Kbp',Kbp,'dp_crack',dp_crack,'dp_ind',dp_ind,'Lf',0.05,'Af',1e-3, ...
    'Vh',Vh,'Vo',Vo,'UA',2);
end

function s = mkpoppet(Lp, Dp, dp_crack, R_ovr, CdA_max, Q_rated, fQo, V)
% Q_o   : 크래킹 유량 스케일 = fQo*Q_rated  (크래킹 이하 차압에서의 미소 누설 수준)
%         너무 작게(<AbsTol) 잡으면 암시적 솔버의 Newton 반복이 잘못 수렴하므로 주의
% R_rev : 역방향 누설 저항 (0.1 L/min @ 3000 psid)
s = struct('Lp',Lp,'Dp',Dp,'dp_crack',dp_crack,'R_ovr',R_ovr,'CdA_max',CdA_max, ...
    'Q_rated',Q_rated,'Q_o',fQo*Q_rated,'R_rev',20.7e6/(0.1e-3/60),'tau',0.003,'V',V,'UA',0);
end
