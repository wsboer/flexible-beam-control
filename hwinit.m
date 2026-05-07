%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DSCS FPGA interface board: init and I/O conversions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% sample time interval
h = 1e-3;


%% gains and offsets

% sensor gains and offsets
lasergain = 0.2018;
laseroffset =-2.4628;
encodergain = 0.0016;
encoderoffset = -0.625;

adingain = [1 lasergain  encodergain 1 1 1 1];       % input gain
adinoffs = [0 laseroffset encoderoffset 0 0 0 0];    % input offset

% actuator offset
adoutoff = -0.19;

