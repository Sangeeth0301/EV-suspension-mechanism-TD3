function V_x = Lyapunov_Mon(x_state)
% Lyapunov Stability Monitor  V(x) = x_qc' * P * x_qc
% V(x) must be positive and decreasing for Lyapunov stability.
% P is the solution to CARE with Q=diag([1e5,1e4,1e4,1e2]), R=1e-3.
a  = 1.10;
Pl = [55514.66518 1292.584418 51939.7711 21.505047;1292.584418 1223.579227 -1867.584611 16.21273818;51939.7711 -1867.584611 68939.36182 -1.11842938;21.505047 16.21273818 -1.11842938 2.749362062];

% Map half-car state to front quarter-car state vector
xq  = [x_state(1)-a*x_state(2)-x_state(3); ...
        x_state(5)-a*x_state(6); 0; x_state(7)];
V_x = xq' * Pl * xq;
end