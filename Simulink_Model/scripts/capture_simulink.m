% capture_simulink.m
clear; clc;
try
    disp('Loading parameters (running setup)...');
    create_simulink_model;
    
    disp('Exporting Simulink diagram to PNG...');
    % Print the block diagram
    print('-sCyberSuspension_MultiBlock', '-dpng', 'simulink_model.png');
    disp('Saved to simulink_model.png');
catch ME
    disp(ME.message);
end
exit;
