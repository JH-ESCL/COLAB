function S = hyd_sys_index(P)
%HYD_SYS_INDEX  ODE 레퍼런스 모델용 상태 인덱스 / 연결 맵 생성
topo = P.topo;
nb = numel(topo.blocks);
names = {topo.blocks.name};
S.P = P; S.nb = nb; S.names = names;
ofs = 0; X0 = []; atol = [];
for k = 1:nb
    bk  = topo.blocks(k);
    prm = P.blk.(bk.name);
    n   = numel(prm.x0);
    S.blk(k).name = bk.name;
    S.blk(k).type = bk.type;
    S.blk(k).idx  = ofs + (1:n);
    S.blk(k).prm  = prm;
    S.blk(k).Nin  = bk.Nin;
    S.blk(k).Nout = bk.Nout;
    S.blk(k).extId = bk.extId;
    S.blk(k).f = str2func(['lru_' bk.type '_f']);
    S.blk(k).g = str2func(['lru_' bk.type '_g']);
    S.blk(k).up = zeros(bk.Nin, 2);   % [상류 블록, 상류 출구번호] (입구별)
    S.blk(k).dn = zeros(bk.Nout, 2);  % [하류 블록, 하류 입구번호] (출구별)
    ofs  = ofs + n;
    X0   = [X0; prm.x0(:)];   %#ok<AGROW>
    atol = [atol; prm.atol(:)]; %#ok<AGROW>
end
for c = 1:numel(topo.conns)
    cn = topo.conns(c);
    a = find(strcmp(names, cn.from)); d = find(strcmp(names, cn.to));
    S.blk(a).dn(cn.out, :) = [d, cn.in];
    S.blk(d).up(cn.in,  :) = [a, cn.out];
end
for k = 1:nb   % 연결 누락 검사
    if any(S.blk(k).up(:) == 0) || any(S.blk(k).dn(:) == 0)
        error('hyd_sys_index:conn', 'Block %s has unconnected ports', S.blk(k).name);
    end
end
S.X0 = X0; S.atol = atol; S.nx = ofs;
end
