function mpc = case3_acdc()
%case 3 nodes    Power flow data for 3 bus, 2 generator case.
%   Please see 'help caseformat' for details on the case file format.
%

%% MATPOWER Case Format : Version 1
%%-----  Power Flow Data  -----%%
%% system MVA base
mpc.baseMVA = 100;

%% bus data
%	bus_i	type	Pd	Qd	Gs	Bs	area	Vm      Va	baseKV	zone	Vmax	Vmin
mpc.bus = [
	1       3       20	10	0   0   1       1.06	0	345     1       1.1     0.9;
	2       2       0	0	0   0   1       1       0	345     1       1.1     0.9;
	3       1       45	15	0   0   1       1       0	345     1       1.1     0.9;
];

%% generator data
%	bus	Pg      Qg	Qmax	Qmin	Vg	mBase       status	Pmax	Pmin	pc1 pc2 qlcmin qlcmax qc2min qc2max ramp_agc ramp_10 ramp_30 ramp_q apf
mpc.gen = [
	1	40      0	500      -500    1.06	100       1       250     0 0 0 0 0 0 0 0 0 0 0 0;
    2	40      0	300      -300    1      100       1       300     0 0 0 0 0 0 0 0 0 0 0 0;
];

%% branch data
%	fbus	tbus	r	x	b	rateA	rateB	rateC	ratio	angle
%	status angmin angmax
mpc.branch = [
    1   2   0.02    0.06    0.06    100   100   100     0       0       1 -60 60;
    2   3   0.06    0.18    0.04    100   100   100     0       0       1 -60 60;
];


%% dc grid topology
%colunm_names% dcpoles
mpc.dcpol=2;
% numbers of poles (1=monopolar grid, 2=bipolar grid)
%% bus data
%column_names%   busdc_i grid    Pdc     Vdc     basekVdc    Vdcmax  Vdcmin  Cdc
mpc.busdc = [
    1              1       0       1       345         1.1     0.9     0;
    2              1       0       1       345         1.1     0.9     0;
	3              1       0       1       345         1.1     0.9     0;
];

%% converters
%column_names%   busdc_i busac_i type_dc type_ac P_g   Q_g islcc  Vtar    rtf xtf  transformer tm   bf filter    rc      xc  reactor   basekVac    Vmmax   Vmmin   Imax    status   LossA LossB  LossCrec LossCinv  droop      Pdcset    Vdcset  dVdcset Pacmax Pacmin Qacmax Qacmin
mpc.convdc = [
    1       1   1       1       -60    -40    0 1     0.01  0.01 1 1 0.01 1 0.01   0.01 1  345         1.1     0.9     1.1     1       1.103 0.887  2.885    2.885      0.0050    -58.6274   1.0079   0 100 -100 50 -50;
    2       2   2       1       0       0     0 1     0.01  0.01 1 1 0.01 1 0.01   0.01 1  345         1.1     0.9     1.1     1       1.103 0.887  2.885    2.885      0.0070     21.9013   1.0000   0 100 -100 50 -50;
    3       3   1       1       35       5    0 1     0.01  0.01 1 1 0.01 1 0.01   0.01 1  345         1.1     0.9     1.1     1       1.103 0.887  2.885    2.885      0.0050     36.1856   0.9978   0 100 -100 50 -50;
];

%% branches
%column_names%   fbusdc  tbusdc  r      l        c   rateA   rateB   rateC   status
mpc.branchdc = [
    1       2       0.052   0   0    100     100     100     1;
    2       3       0.052   0   0    100     100     100     1;
];

%% generator cost data
%	1	startup	shutdown	n	x1	y1	...	xn	yn
%	2	startup	shutdown	n	c(n-1)	...	c0
mpc.gencost = [
	2	0	0	3	100    1 	1;
	2	0	0	3   0	   2	0;
];

% adds current ratings to branch matrix
%column_names%	c_rating_a
mpc.branch_currents = [
100;100;
];
