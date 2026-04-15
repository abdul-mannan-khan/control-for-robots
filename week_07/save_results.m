%% Week 7: Backstepping Control Simulation with Saved Results
fprintf('=== Week 7: Backstepping Control Simulation with Saved Results ===\n\n');

% Run the main simulation
run('backstepping_control_simulation.m');

% Define output directory AFTER the simulation
output_dir = '/home/it-services/auto_control_ws/results/week_06';

% Get all figure handles and save
fig_handles = findall(0, 'Type', 'figure');
for i = 1:length(fig_handles)
    fig = fig_handles(i);
    fig_name = get(fig, 'Name');
    if isempty(fig_name)
        fig_name = sprintf('figure_%d', fig.Number);
    end
    fig_name = regexprep(fig_name, '[^a-zA-Z0-9_\-]', '_');
    fig_name = regexprep(fig_name, '_+', '_');
    saveas(fig, fullfile(output_dir, [fig_name '.png']));
    saveas(fig, fullfile(output_dir, [fig_name '.fig']));
    fprintf('  Saved: %s.png\n', fig_name);
end

fprintf('\n=== Week 7 Backstepping Control Results Saved ===\n');
fprintf('Output directory: %s\n', output_dir);
