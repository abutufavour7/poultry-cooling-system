s = tf('s');

% K = -0.0081 
% L = 1.0 min = 60 seconds
% tau = 4.5 min = 270 seconds
G = (-0.0081 * exp(-60*s)) / (270*s + 1);

opts = pidtuneOptions('PhaseMargin', 45);
[C, info] = pidtune(G, 'PID', opts)