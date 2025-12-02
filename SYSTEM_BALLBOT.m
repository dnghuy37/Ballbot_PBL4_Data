clear;
Parameter;

A_hat = [ A	zeros(size(B))
	    Cy	0 ]; 

B_hat = [   B
	        0	];
Ck = [1 0 0 0
      0 0 0 1];
Dk = [0;0];

Wd = 2000;
Wn = 0.005;

% DaiCanh = 500;
% DoCao = sqrt(3) / 2 * DaiCanh;
% TimeCanh = 0.2;
% TimeDelay = 3.4;
% TimeBatDau = 10;

% DaiCanh = 800;
% TimeCanh = 1;
% TimeDelay = 3.4;
% TimeBatDau = 10;

% DaiCanh = 800;
% DoCao = sqrt(3) / 2 * DaiCanh;
% TimeCanh = 0.2;
% TimeCanhBen = 1;
% TimeDelay = 3.2;
% TimeBatDau = 10;


% Q = [   10     0	   0	  0	    0
%         0	   100000  0      0	    0	
%         0	   0	   1	  0	    0	
%         0	   0	   0	  1	    0	
%         0      0	   0	  0	    126185.688];
% R = 1e10;


% T??ng t? v?i b�n Yamamoto
Q = [   10     0	   0	  0	    0
        0	   100000  0      0	    0	
        0	   0	   1	  0	    0	
        0	   0	   0	  1	    0	
        0      0	   0	  0	    584185.688];
R = 1e10;


% Q = [   10     0	   0	  0	    0
%         0	   100000  0      0	    0	
%         0	   0	   1	  0	    0	
%         0	   0	   0	  1	    0	
%         0      0	   0	  0	    126185.68831];
% R = 1e10;



% Q = [   100 	   0	 0	    0	    0
% 	    0	       1e5   0      0	    0	
% 	    0	       0	 1	    0	    0	
% 	    0	       0	 0	    1	    0	
% 	    0          0	 0	    0	    5336.69923];
% 
% R = 1e8;

% Q = [   1 	   0	 0	    0	    0   
% 	    0	       6e5  0      0	    0   
% 	    0	       0	 1	    0	    0   
% 	    0	       0	 0	    1	    0   
% 	    0          0	 0	    0	    1e3 ];
% 
% R = 6e3*(180/pi)^2;


% Q = [   30000 	       0	 0	    0	    0
% 	    0	       600   0      0	    0	
% 	    0	       0	 0.01	0	    0	
% 	    0	       0	 0	    1	    0	
% 	    0          0	 0	    0	    100 ];
% 
% R = 100000697;

%break poles
% Q = [   300 	   0	 0	    0	    0
% 	    0	       600   0      0	    0	
% 	    0	       0	 1	    0	    0	
% 	    0	       0	 0	    1	    0	
% 	    0          0	 0	    0	    1 ];
% 
% R= 1e6;

% Q = [   30 	   0	 0	    0	    0
% 	    0	       600   0      0	    0	
% 	    0	       0	 0.01	0	    0	
% 	    0	       0	 0	    100	 0	
% 	    0          0	 0	    0	    0.2 ];
% 
% R = 20000;

%Chỉnh R đáp ứng sẽ tăng giảm dần

% Q = [   30 	   0	 0	    0	    0
% 	    0	       600   0      0	    0	
% 	    0	       0	 0.001	    0	    0	
% 	    0	       0	 0	    100	    0	
% 	    0          0	 0	    0	    100 ];
% 
% R = 1e9;

% %All 1
% Q = [   1 	   0	 0	    0	    0
% 	    0	   600   0      0	    0	
% 	    0	   0	 0.001	0	    0	
% 	    0	   0	 0	    0.1	    0	
% 	    0      0	 0	    0	    0.1 ];
% 
% R = 19697;

% %All 1
% Q = [   10 	       0	 0	    0	    0
% 	    0	       600   0      0	    0	
% 	    0	       0	 1	    0	    0	
% 	    0	       0	 0	    1	    0	
% 	    0          0	 0	    0	    1 ];
% 
% R = 19697;

% Q = [   0.206943489832327	0	0	0	0
% 0	13505.3544100484	0	0	0
% 0	0	0.00100000000000000	0	0
% 0	0	0	0.00100000000000000	0
% 0	0	0	0	1 ];
% 
% R = 1.8545e+04;

% Q = [96.3464852074739	0	0	0	0
% 0	5959.20702841507	0	0	0
% 0	0	0.00100000000000000	0	0
% 0	0	0	0.00100000000000000	0
% 0	0	0	0	1];
% 
% R = 19195.9377850085;

% Q = [293.804098678844	0	0	0	0
% 0	1422.40240115499	0	0	0
% 0	0	0.00100000000000000	0	0
% 0	0	0	0.00100000000000000	0
% 0	0	0	0	1];
% R = 1.9587e+04;

% Q = [293.804098678844	0	0	0	0
% 0	2214.81833397159	0	0	0
% 0	0	0.00100000000000000	0	0
% 0	0	0	0.00100000000000000	0
% 0	0	0	0	1];
% R = 19509.8346076300;

%R = 6e3*(180 / pi)^2;
%TS=0.0033;
TS = 0.004;
ts1 = 0.004;
% Q = [   1 	0	0	0	0
% 	    0	800 0   0	0	
% 	    0	0	1	0	0	
% 	    0	0	0	1	0	
% 	    0	0	0	0	100 ];
% 
% R = 20000;

Q_nor = [   1 	0	0	0	
	        0	6e5 0   0		
	        0	0	1	0		
	        0	0	0	1		];
    
R_nor = 6e3*(180 / pi)^2;



K_lqr = lqr(A_hat, B_hat, Q, R);
% K_lqr(3)=K_lqr(3)+0.005;
% K_lqr(4)=K_lqr(4)+0.001;
K_x = lqr (A,B,Q_nor,R_nor);
k_f = K_lqr(1:4);
k_i = K_lqr(5); %-


% Q = [   1e3 	0	0	0	
% 	    0	6e5	0	0		
% 	    0	0	1	0		
% 	    0	0	0	1];
% 
% R = 6e3*(180 / pi)^2;
% 
% K_lqr = lqr(A, B, Q, R);
% k_f=K_lqr(1:4);
% k_i=0;



state_cl = {'\theta [\circ]' '\psi [\circ]' 'd\theta/dt [\circ/s]'...
            'd\psi/dt [\circ/s]' 'error'};   
input_cl = {'r'};
output_cl = {'\theta [\circ]' '\psi [\circ]' 'd\theta/dt [\circ/s]'...
            'd\psi/dt [\circ/s]' };   
%-B*K_lqr(5)
sys_clt = ss([A-B*K_lqr(1:4) B*K_lqr(5); -C(1,:) 0],[zeros(size(B));1],...
            [C zeros(size(B))],D,'statename',state_cl,...
            'inputname',input_cl,'outputname',output_cl);


% sys = ss(A,B,Ck,Dk);
% %sys = ss([A-B*K_lqr(1:4) -B*K_lqr(5); -C(1,:) 0],[zeros(size(B));1],...
% %            [C zeros(size(B))],D);
% onyl_lqr_sys= ss(A,B,C,D);
% dis_sys = c2d(sys,0.004);
% %K_lqr_dis = dlqr(dis_sys.A,dis_sys.B,Q,R);
% only_lqr_dis_sys=c2d(onyl_lqr_sys,0.004);
%k_f_dis = K_lqr_dis(1:4);
%k_i_dis = -K_lqr_dis(5);
% Q = 0.1*eye(4);
% R = eye(2);
% 
% [Kf,L,P] = lqe(A,Q,Ck,R,Q);



%Stability analysis

pole_closeloop = pole(sys_clt)


[wn,zeta] = damp(sys_clt)

% % Simulating System Response 
% t = 0:0.004:40;
% % System response to a step input 
% figure2 = figure('Color',[1 1 1]);
% axes1 = axes('Parent',figure2,'FontSize',14,'FontName','Times New Roman');
% box(axes1,'on');
% step(100*sys_clt,t);
% y_step = step(100*sys_clt,t);
% theta_response = y_step(:,1,1);
% info_step = stepinfo(theta_response, t, 100);  % assuming target value = 1
% 
% 
% x0 = [0; 4; 0; 0; 0];  % Initial state [theta, psi, dtheta/dt, dpsi/dt, error]
% t = 0:0.004:10;
% initial(sys_clt, x0, t);
% [y, t_out, x] = initial(sys_clt, x0, t);
% theta_response = y(:,1);  % theta output
% final_value = theta_response(end);  % steady state value (for comparison)
% % Control signal u = -K * x_hat (from K_lqr)
% x_hat = x;
% u_ctrl = -K_lqr * x_hat';
% max_controls = max(abs(u_ctrl))
% all_torque = 100*Kp*(u_ctrl-Kb*Rs*(x(:,3)'-x(:,4)')*pi/(Rw*180))/Ra;
% max_torque_N_cm = max(abs(all_torque))
% % Use stepinfo for transient metrics (with final value as reference)
% info_ini = stepinfo(theta_response, t_out, final_value);


%Tham so mo phong

TIME_START=100;
pwm_gain=1; %ProfitApwm
pwm_offset=0; %offsetpwm
a_b=0.8; %AverageBatteryValue
a_d=0.8; %noise suppressor in speed
a_r=0.996; %Soft Reference Signal
a_gc= 0.8; %OffsetGyro Sensor Calibration
a_gd= 0.999; %Rotation sensor drift compensation
k_thetadot=0.12/Rs*180/pi; %Speedgain(0.12[m/sec])
%psi_dif_umb=45; %threshold of the angle of inclination of the body[degrees]
%theta_dif_umb=4*360;%Threshold of the rolling angle of the sphere[degrees]

GYRO0=600; %Initial Value of the Gyroscopic Sensor
BATTERY=8000; %initial battery value[mV]
PSI0= 4; %Initial value of the pitch angle of the body [degrees]
XIV=[ %Initial state
0
PSI0
0
0
];
%Sampling rates
%TS=0.004;

%ts1=0.004; % ts1 0.004 chay chung voi TS 0.0033
START_POS=[0,0];
%Conditions for the Detention of the Simulation
BODY_ANGLE_MAX=35; %Maximum Body Angle
BODY_ANGLE_MIN=-35; %Minimum Body Angle

% psi = reshape(out.psi,[7908,1]);
% psidot = reshape(out.psidot,[7908,1]);
% theta = reshape(out.theta,[7908,1]);
% thetadot = reshape(out.thetadot,[7908,1]);
% vol = reshape(out.vol,[7908,1]);

Circle_gain = 15;
Radius = 500;