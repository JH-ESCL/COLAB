%HYD_INIT  유압 시스템 시뮬레이션 경로 설정
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'lib', 'common'), fullfile(root, 'lib', 'lru'), ...
        fullfile(root, 'params'), fullfile(root, 'build'), fullfile(root, 'sim'));
clear root
