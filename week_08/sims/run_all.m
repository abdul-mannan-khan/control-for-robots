function run_all()
% Master runner: executes all 5 controllers' optimization demos on
% manipulator, mobile robot, quadrotor. Saves PNGs + summary.

set(groot,'defaultFigureVisible','off');
here = fileparts(mfilename('fullpath'));
figdir = fullfile(here,'figures');
if ~exist(figdir,'dir'), mkdir(figdir); end

tic;
R.smc         = sim_smc(true);
R.adaptive    = sim_adaptive(true);
R.robust      = sim_robust(true);
R.backstep    = sim_backstepping(true);
R.nmpc        = sim_nmpc(true);
elapsed = toc;

fprintf('\n================= SUMMARY =================\n');
fprintf('Total wall time: %.1f s\n\n', elapsed);
ctrls={'smc','adaptive','robust','backstep','nmpc'};
robots={'manipulator','mobile','quad'};
fprintf('%-12s %-12s %-12s %-12s %-10s\n','Controller','Robot','J_baseline','J_opt','Reduction');
fprintf('--------------------------------------------------------------------\n');
for i=1:numel(ctrls)
    for j=1:numel(robots)
        r = R.(ctrls{i}).(robots{j});
        if isfield(r,'J0')
            red = 100*(r.J0 - r.JOpt)/max(r.J0,eps);
            fprintf('%-12s %-12s %-12.4f %-12.4f %+7.1f%%\n',ctrls{i},robots{j},r.J0,r.JOpt,red);
        elseif isfield(r,'J_short')
            red = 100*(r.J_short - r.J_long)/max(r.J_short,eps);
            fprintf('%-12s %-12s %-12.4f %-12.4f %+7.1f%%  (short vs long horizon)\n',...
                ctrls{i},robots{j},r.J_short,r.J_long,red);
        end
    end
end
save(fullfile(here,'results.mat'),'R','elapsed');
fprintf('\nFigures saved to: %s\n', figdir);
fprintf('Results MAT file: %s\n', fullfile(here,'results.mat'));
end
