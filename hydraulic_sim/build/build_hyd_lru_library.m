function lib = build_hyd_lru_library(lib)
%BUILD_HYD_LRU_LIBRARY  LRU 요소 Simulink 라이브러리(hyd_lru_lib.slx) 생성
%   각 LRU 타입별 대표 블록 1개 (포트 수는 대표 인스턴스 기준).
%   마스크 'prm' 을 다른 P.blk.<NAME> 으로 바꿔 재사용.
%   입구/출구 수가 다른 구성은 hyd_add_lru(sys,name,type,Nin,Nout,hasExt,prmExpr,pos) 로 생성.
if nargin < 1, lib = 'hyd_lru_lib'; end
P = hyd_params();
assignin('base', 'P', P);
topo = P.topo;
if bdIsLoaded(lib), close_system(lib, 0); end
load_system('simulink');
new_system(lib, 'Library');
done = {};
col = 0;
for k = 1:numel(topo.blocks)
    bk = topo.blocks(k);
    if any(strcmp(done, bk.type)), continue; end
    done{end+1} = bk.type; %#ok<AGROW>
    r = floor(col/4); c = mod(col, 4); col = col + 1;
    pos = [40 + c*220, 40 + r*180, 170 + c*220, 150 + r*180];
    hyd_add_lru(lib, upper(bk.type), bk.type, bk.Nin, bk.Nout, bk.extId > 0, ...
        ['P.blk.' bk.name], pos);
end
save_system(lib);
fprintf('Library "%s" built: %s\n', lib, strjoin(done, ', '));
end
