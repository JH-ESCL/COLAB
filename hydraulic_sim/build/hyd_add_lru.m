function blk = hyd_add_lru(sys, name, type, Nin, Nout, hasExt, prmExpr, pos)
%HYD_ADD_LRU  LRU 1개를 마스크 서브시스템으로 생성
%
%  내부 구조 (모든 LRU 공통):
%     in<i>_fwd  ─┐                                       ┌─> in<i>_bwd   [Q;T]  (상류로)
%                Mux─> F: [dx,mon]=lru_<type>_f(...) ─dx─> 1/s (X) ─x─> G: [fwd,bwd]=lru_<type>_g(x)
%     out<k>_bwd ─┘          ^ x                                         └─> out<k>_fwd [P;T]  (하류로)
%     ext ───────────────────┘                     mon ─> mon
%
%  * G 는 상태 x 만의 함수 -> LRU 간 대수루프가 생기지 않음
%  * 마스크 파라미터 'prm' (예: P.blk.EDP) -> F/G 의 non-tunable 파라미터, Integrator IC = prm.x0
%
%  포트 번호:  Inport  : in1_fwd..inN_fwd, out1_bwd..outM_bwd, (ext)
%              Outport : in1_bwd..inN_bwd, out1_fwd..outM_fwd, mon
blk = [sys '/' name];
add_block('built-in/Subsystem', blk, 'Position', pos);

% ---------- 포트 ----------
y = 30; dy = 40; ip = 0; op = 0;
for i = 1:Nin
    ip = ip + 1;
    add_block('built-in/Inport', sprintf('%s/in%d_fwd', blk, i), 'Port', num2str(ip), ...
        'Position', [20 y 50 y+14]); y = y + dy;
end
for k = 1:Nout
    ip = ip + 1;
    add_block('built-in/Inport', sprintf('%s/out%d_bwd', blk, k), 'Port', num2str(ip), ...
        'Position', [20 y 50 y+14]); y = y + dy;
end
if hasExt
    ip = ip + 1;
    add_block('built-in/Inport', [blk '/ext'], 'Port', num2str(ip), 'Position', [20 y 50 y+14]);
else
    add_block('built-in/Constant', [blk '/ext'], 'Value', 'zeros(4,1)', 'Position', [20 y 50 y+20]);
end
y = 30;
for i = 1:Nin
    op = op + 1;
    add_block('built-in/Outport', sprintf('%s/in%d_bwd', blk, i), 'Port', num2str(op), ...
        'Position', [720 y 750 y+14]); y = y + dy;
end
for k = 1:Nout
    op = op + 1;
    add_block('built-in/Outport', sprintf('%s/out%d_fwd', blk, k), 'Port', num2str(op), ...
        'Position', [720 y 750 y+14]); y = y + dy;
end
op = op + 1;
add_block('built-in/Outport', [blk '/mon'], 'Port', num2str(op), 'Position', [720 y+40 750 y+54]);

% ---------- 입력 Mux ----------
if Nin > 1
    add_block('built-in/Mux', [blk '/MuxFwd'], 'Inputs', num2str(Nin), 'Position', [120 30 125 30+30*Nin]);
    for i = 1:Nin, add_line(blk, sprintf('in%d_fwd/1', i), sprintf('MuxFwd/%d', i)); end
    srcFwd = 'MuxFwd/1';
else
    srcFwd = 'in1_fwd/1';
end
if Nout > 1
    add_block('built-in/Mux', [blk '/MuxBwd'], 'Inputs', num2str(Nout), 'Position', [120 150 125 150+30*Nout]);
    for k = 1:Nout, add_line(blk, sprintf('out%d_bwd/1', k), sprintf('MuxBwd/%d', k)); end
    srcBwd = 'MuxBwd/1';
elseif Nout == 1
    srcBwd = 'out1_bwd/1';
else   % 하류 연결이 없는 LRU (예: 어큐뮬레이터)
    add_block('built-in/Constant', [blk '/NoDownstream'], 'Value', '[0; prm.x0(end)]', ...
        'Position', [100 150 160 170]);
    srcBwd = 'NoDownstream/1';
end

% ---------- F / Integrator / G ----------
fF = [blk '/F']; fG = [blk '/G'];
add_block('simulink/User-Defined Functions/MATLAB Function', fF, 'Position', [260 40 400 140]);
add_block('built-in/Integrator', [blk '/X'], 'InitialCondition', 'prm.x0', ...
    'AbsoluteTolerance', 'prm.atol', 'Position', [450 70 480 100]);
add_block('simulink/User-Defined Functions/MATLAB Function', fG, 'Position', [540 40 660 140]);

setMLFcn(fF, sprintf(['function [dx, mon] = F(x, fwd_in, bwd_in, ext, prm)\n' ...
    '%%#codegen\n[dx, mon] = lru_%s_f(x, fwd_in, bwd_in, ext, prm);\n'], type));
setMLFcn(fG, sprintf(['function [fwd_out, bwd_out] = G(x, prm)\n' ...
    '%%#codegen\n[fwd_out, bwd_out] = lru_%s_g(x, prm);\n'], type));

add_line(blk, 'X/1', 'F/1', 'autorouting', 'on');
add_line(blk, srcFwd, 'F/2', 'autorouting', 'on');
add_line(blk, srcBwd, 'F/3', 'autorouting', 'on');
add_line(blk, 'ext/1', 'F/4', 'autorouting', 'on');
add_line(blk, 'F/1', 'X/1', 'autorouting', 'on');
add_line(blk, 'X/1', 'G/1', 'autorouting', 'on');
add_line(blk, 'F/2', 'mon/1', 'autorouting', 'on');

% ---------- 출력 Demux ----------
if Nout > 1
    add_block('built-in/Demux', [blk '/DemuxFwd'], 'Outputs', mat2str(2*ones(1, Nout)), ...
        'Position', [690 150 695 150+30*Nout]);
    add_line(blk, 'G/1', 'DemuxFwd/1', 'autorouting', 'on');
    for k = 1:Nout, add_line(blk, sprintf('DemuxFwd/%d', k), sprintf('out%d_fwd/1', k)); end
elseif Nout == 1
    add_line(blk, 'G/1', 'out1_fwd/1', 'autorouting', 'on');
else
    add_block('built-in/Terminator', [blk '/TermFwd'], 'Position', [690 60 710 80]);
    add_line(blk, 'G/1', 'TermFwd/1', 'autorouting', 'on');
end
if Nin > 1
    add_block('built-in/Demux', [blk '/DemuxBwd'], 'Outputs', mat2str(2*ones(1, Nin)), ...
        'Position', [690 30 695 30+30*Nin]);
    add_line(blk, 'G/2', 'DemuxBwd/1', 'autorouting', 'on');
    for i = 1:Nin, add_line(blk, sprintf('DemuxBwd/%d', i), sprintf('in%d_bwd/1', i)); end
else
    add_line(blk, 'G/2', 'in1_bwd/1', 'autorouting', 'on');
end

% ---------- 마스크 ----------
m = Simulink.Mask.create(blk);
m.Type = ['Hydraulic LRU: ' upper(type)];
m.Description = sprintf(['%s (type = %s)\n상태/수식: help lru_%s_f\n' ...
    '신호 규약: fwd=[P(Pa abs); T(degC)] 상류->하류, bwd=[Q(m^3/s); T(degC)] 하류->상류'], ...
    name, type, type);
m.addParameter('Type', 'edit', 'Name', 'prm', 'Prompt', 'LRU 파라미터 구조체', 'Value', prmExpr);
m.Display = sprintf('disp(''%s\\n[%s]'')', name, type);
set_param(blk, 'BackgroundColor', lruColor(type));
end

% =========================================================================
function setMLFcn(path, script)
rt = sfroot;
ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', path);
ch.Script = script;
d = ch.find('-isa', 'Stateflow.Data', 'Name', 'prm');
d.Scope = 'Parameter';
try
    d.Tunable = false;   % 구조체 필드(Nin/Nout)를 배열 크기로 사용 -> non-tunable
catch
end
end

function c = lruColor(type)
switch type
    case {'edp','acmp'},           c = 'orange';
    case {'rsv','acc'},            c = 'cyan';
    case {'filter'},               c = 'lightBlue';
    case {'ckv','rv','sov'},       c = 'yellow';
    case {'act'},                  c = 'green';
    case {'hx'},                   c = 'magenta';
    otherwise,                     c = 'white';
end
end
