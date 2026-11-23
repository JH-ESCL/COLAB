# 유압 시스템 Simulink 모델 (상태변수: 압력 P · 유량 Q · 온도 T)

항공기 유압 계통 1개(3000 psi급)를 **LRU 단위 요소**로 나눠 만든 시스템 수준 열-유압 모델입니다.
Simscape를 쓰지 않고 **Simulink 기본 블록과 MATLAB Function 블록만** 사용합니다.
같은 LRU 함수를 쓰는 **순수 MATLAB(ode15s) 레퍼런스 모델**도 함께 들어 있습니다.

```
hydraulic_sim/
├─ hyd_init.m                 경로 설정
├─ lib/common/                공통 물리 함수 (유체 물성, 손실, 노드, 펌프·포펫 코어)
├─ lib/lru/                   LRU 요소: lru_<type>_f.m (미분식) / lru_<type>_g.m (출력식)
├─ params/                    hyd_params(LRU 파라미터), hyd_topology(연결), hyd_scenario(운용 입력)
├─ build/                     Simulink 자동 생성: build_hyd_system, build_hyd_lru_library, hyd_add_lru
└─ sim/                       run_hyd_sim(Simulink), run_hyd_ode(MATLAB ODE), hyd_plot_results
```

## 사용법

```matlab
cd hydraulic_sim
hyd_init
build_hyd_lru_library      % hyd_lru_lib.slx : LRU 타입별 블록 라이브러리 (선택)
build_hyd_system           % hyd_system.slx  : 19개 LRU, 23개 배관 연결
run_hyd_sim                % 시뮬레이션 + 플롯 (Simulink 없이 돌리려면 run_hyd_ode)
```

솔버 설정과 주의사항:
- `ode15s`, **MaxOrder 2**, **RelTol 1e-4**. 상태별 AbsTol(`prm.atol`)은 각 Integrator에 줍니다.
- AbsTol 기본값(`hyd_lru_init`)은 Q 1e-7 m³/s, P 100 Pa, T 1e-3 K 입니다.
- 이보다 느슨하게(예: RelTol 1e-3, Q 1e-6) 풀면 수요가 0인 정지 상태(예: 액추에이터 스텝 유지)에서 문제가 생깁니다.
  체크밸브 유량이 크래킹 스케일 `Q_o`(0.4–1.4 L/min) 근처에서 진동하는데, 이때 Newton이 엉뚱한 해로 넘어갑니다.
  그러면 체크밸브 slamming, 9000 psi 압력 스파이크, 릴리프 개방 같은 가짜 진동이 생깁니다.
  1e-5로 엄격하게 푼 해와 비교해서 수치 오류임을 확인했습니다.
- 밸브 Δp(Q) 특성은 Q = 0에서 값과 기울기가 모두 연속(C¹)이어야 합니다(`hyd_poppet_core` 참고).
  새 밸브 LRU를 만들 때도 이 조건을 지키세요.
- 작은 체적과 짧은 관성 때문에 감쇠가 작은 고주파 모드(수 kHz)가 많아 **stiff 솔버가 필요**합니다.
- **MaxOrder를 3 이상으로 두면 안 됩니다.** 3차 이상 BDF/NDF는 A‑안정이 아니라서 이런 모드를 수치적으로
  키우고, 결국 체크밸브 slamming과 비슷한 가짜 진동이 생깁니다. 검증 중 실제로 확인했습니다.
- AbsTol을 너무 작게 잡으면(예: Q 1e-8 m³/s) 스텝이 수천 배 작아집니다.
- 밸브의 크래킹 유량 스케일 `Q_o`는 AbsTol보다 충분히 커야 합니다. 너무 작으면 Newton 반복이 잘못 수렴합니다.

## 모델링 방법

### 1) LRU 하나 = "관성 입구 + 용량성 노드" (R‑L‑C 세그먼트)

모든 LRU는 같은 구조의 상태를 가집니다.

| 상태 | 식 | 의미 |
|---|---|---|
| **Q** (입구별) | ρL/A · dQ/dt = P_up − P − Δp_loss(Q, T) + Δp_gain | 유체 관성, 손실은 점도 ν(T)에 따라 변함 |
| **P** (노드별) | V/β(P,T) · dP/dt = ΣQ_in − ΣQ_out | 압축성 (혼입공기 포함 유효 β) |
| **T** (노드별) | ρc_pV · dT/dt = ρc_p Σ Q⁺(T_in − T) + Q̇_loss − UA(T − T_amb) | 대류 수송 + **교축/마찰 발열** + 방열 |

여기서 Q̇_loss = Δp·Q 입니다. 오리피스·필터·배관 손실이 그대로 유체 온도 상승으로 이어지므로
**압력–유량–온도가 서로 영향을 줍니다** (T → 점도 → 손실·누설 → 발열 → T).

### 2) 신호 규약 (대수 루프 없음)

배관 연결 하나마다 신호 2개가 오갑니다.

```
 상류 LRU ── fwd = [P; T] ──▶ 하류 LRU      (상류 노드 압력·온도)
          ◀── bwd = [Q; T] ──              (하류 입구 유량 + 역류 시 쓸 온도)
```

각 LRU 서브시스템 안에는 `F`(미분식) → `Integrator` → `G`(출력식) 블록이 있습니다.
`G`는 **상태 x만의 함수**라서 LRU를 어떻게 연결해도 대수 루프가 생기지 않습니다.
입구·출구가 여러 개인 LRU(저장조, 매니폴드, 필터, 펌프)는 fwd/bwd를 `[P1;T1;P2;T2…]`처럼 쌓아서 받습니다.

## LRU 요소 목록

| LRU | 타입(함수) | 상태 x | 주요 물리 | ext 입력 |
|---|---|---|---|---|
| **RSV** 저장조 (공기 가압식) | `rsv` | Q_1..Q_n, V_fl, P_g, T | 레귤레이터 가압 P_g, 유면 수두 ρgh, 완전혼합 온도 | – |
| **SOV** 공급 차단밸브 | `sov` | Q, P, T, s | 모터 구동 개도 s(속도 제한), 오리피스 CdA(s) | 개폐 명령 |
| **SUPL/PLINE/RLINE** 배관 | `line` | Q, P, T | Darcy(층류/난류 Blasius) + 국부손실 K, 자연대류 방열 | – |
| **EDP** 엔진 구동 펌프 | `edp` | Q_s, P_in, T_in, ε, P_out, T_out, P_c, T_c | 압력보상 사판 ε, 점도 의존 내부누설 → 케이스, 기계손실 발열, 캐비테이션 충진계수 | 회전수 N |
| **ACMP** 교류 모터 펌프 | `acmp` | EDP와 같음 + ω | 유도전동기 Kloss 토크–슬립, J·dω/dt | 전원 ON/OFF |
| **PFM/RTF/CDF** 필터 모듈 | `filter` | Q_1..Q_n, P_h, T_h, Q_f, P_o, T_o | 다공질 층류 Δp ∝ μ(T)·막힘도, 바이패스 밸브, 차압 지시기(DPI) | – |
| **CKV** 체크밸브 | `ckv` | Q, P, T, o | 단조 Δp(Q) 특성: 크래킹 5 psid + 전개 오리피스, 역류 시 고저항 누설 | – |
| **RV** 릴리프 밸브 | `rv` | Q, P, T, o | 3500 psid 크래킹 + 오버라이드 150 psi@140 L/min, 릴리프 유량의 교축발열 | – |
| **PMAN** 압력 매니폴드 | `man` | Q_1..Q_n, P, T | 합류 노드 (압력 트랜스듀서 위치) | – |
| **ACC** 어큐뮬레이터 | `acc` | Q, P, T | C(P)·dP/dt = Q, N₂ 폴리트로픽 + 피스톤 바닥 시 잔류체적 압축 | – |
| **ACT** 서보 액추에이터 | `act` | Q_s, P_s, P_A, P_B, P_r, x_p, v_p, x_v, T | ACE 위치루프 + EHSV 4랜드(언더랩 누설), 실린더 챔버, 공력 스프링 하중, 스토퍼 | 위치 명령, 외력 |
| **HX** 열교환기 (연료 냉각) | `hx` | Q, P, T | ε‑NTU 대향류, 튜브 손실 | 연료 유량·온도 |

`help lru_<type>_f`로 각 요소의 수식과 상태 정의를 볼 수 있습니다. 각 LRU의 `mon` 출력 항목도 거기에 적혀 있습니다.

## 시스템 구성 (`params/hyd_topology.m`)

```
RSV ─┬─ SOV ── SUPL_E ── EDP ──(P)── PFM_E ── CKV_E ─┐
     │                    └──(CD)──────────┐          ├─ PMAN ─┬─ ACC
     └─ SUPL_M ─────────── ACMP ─(P)── PFM_M ── CKV_M ┘        ├─ RV ──────────────┐
                            └──(CD)───────┤                    └─ PLINE ─ ACT ─ RLINE ─┤
                                          CDF ── HX ── RSV(CD)                    RTF ── RSV(RET)
```

LRU를 추가하거나 바꾸려면:
1. `lib/lru/lru_<new>_f.m`, `lru_<new>_g.m`를 작성합니다. 시그니처는 그대로 유지합니다:
   `[dx,mon]=f(x,fwd_in,bwd_in,ext,prm)`, `[fwd_out,bwd_out]=g(x,prm)`. g는 x만으로 계산해야 합니다.
2. `params/hyd_lru_init.m`에 x0·atol을 추가합니다.
3. `hyd_params.m`에 파라미터를, `hyd_topology.m`에 블록과 연결을 추가합니다.
4. `build_hyd_system`을 다시 실행합니다.

## 기본 시나리오 (`params/hyd_scenario.m`, 1200 s)

| 시각 | 이벤트 |
|---|---|
| 0–3 s | 엔진 시동, EDP 0 → 2800 rpm (계통 승압, ACC 충전) |
| 10 s~ | 액추에이터 0.5 Hz ±20 mm 사인 운동 |
| 20 s | ACMP ON |
| 300 s | 이륙 출력, EDP 3750 rpm |
| 600–620 s | 액추에이터 40 mm 스텝 유지 |
| 1100 s | SOV CLOSE → EDP 흡입 고갈(캐비테이션) → ACMP 단독 공급 |

## 파라미터와 가정

- 작동유는 MIL‑PRF‑83282 근사입니다 (ρ 832 kg/m³@40 °C, ν 14→3.5 cSt @40→100 °C, c_p 2000 J/kg·K, 혼입공기 0.2 %).
  Skydrol을 쓰려면 `hyd_params.m`의 `fl`만 바꾸면 됩니다.
- 압력은 **절대압 [Pa]**, 온도는 **°C** 입니다. 플롯은 psig로 표시합니다.
- 파라미터는 공개 자료 수준의 대표값입니다 (37 gpm EDP, 10 gpm ACMP, 1 L 어큐뮬레이터 등).
  실제 장비 사양이나 시험 데이터로 교정해야 합니다.
- 1D 집중 파라미터 모델이라 배관 파동(water hammer)은 세그먼트 분할 수준까지만 표현됩니다.
  긴 배관을 정밀하게 보려면 `line` LRU를 여러 개 직렬로 연결하세요.
