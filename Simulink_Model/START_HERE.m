% ========================================================================
% START_HERE.m  -  Run this FIRST in MATLAB / MATLAB Online
% ========================================================================
% This is a thin launcher. All the real work (parameters, CARE gain
% synthesis, stability check, simulation, results table, dashboard figure
% and .slx generation) lives in scripts/build_simulink_model.m.
%
% WHAT THIS DOES
%   1. Adds scripts/ and subsystems/ to the path
%   2. Runs build_simulink_model.m end to end
%   3. Leaves the generated model open so you can press PLAY
%
% WHAT YOU WILL SEE
%   - A console table comparing PASSIVE vs ACTIVE (LPV)
%   - A multi-panel results figure
%   - A Simulink model: scripts/CyberResilient_ActiveSuspension.slx
%
% NOTE: the old CyberSuspension_MultiBlock.slx is SUPERSEDED and is no
% longer opened here. It contains stale controller gains (R = 1e-3 instead
% of 1e-4) and the pre-fix energy model, so its results will not match.
% ========================================================================

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'scripts'));
addpath(fullfile(here, 'subsystems'));

disp('==========================================================');
disp('  Regenerative EV Active Suspension  -  MATLAB Online Setup');
disp('==========================================================');
disp('[*] Running build_simulink_model.m ...');
disp('    (this runs the full simulation AND builds the .slx)');
disp(' ');

build_simulink_model;

disp(' ');
disp('==========================================================');
disp('  DONE.');
disp(' ');
disp('  The console table above is the result. The Simulink model');
disp('  scripts/CyberResilient_ActiveSuspension.slx has also been');
disp('  generated - open it and press PLAY to watch the scopes.');
disp(' ');
disp('  After a Simulink run, call PLOT_RESULTS to tag the graphs.');
disp('==========================================================');
