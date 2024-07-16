%
% Simulate power lock feedback in the Fourier domain
%
clear;
f = logspace(-1,7,1e5);
w = 2*pi*f;

clock_freq = 125e6;
cic_filter_rate = 2^15;
Ts_high = 1./clock_freq;
Ts_low = cic_filter_rate./clock_freq;
zinv_high = exp(-1i*w*Ts_high);
zinv_low = exp(-1i*w*Ts_low);

G0_vv = 4.7;    %V at IN1 / V at AO0
ADC_CONV = 1.1851/(2^(14 - 1));
PWM_CONV = 1.6/(2^10 - 1);
G0 = G0_vv/ADC_CONV*PWM_CONV;

Hpwm = G0*pwm_transfer_function(w);

% These are the digital, integer values
Kp = 00;
Ki = 10;
Kd = 0;
D = 20;
pipeline_delay = 4;

Kp_cont = Kp/2^D;
Ki_cont = Ki/(Ts_low*2^D);
Kd_cont = Kd*Ts_low/(2^D);


K = (Kp_cont*(1 - zinv_low) + Ki_cont/2*(1 + zinv_low) + Kd_cont*(1 - 2*zinv_low + zinv_low.^2))./(1 - zinv_low).*zinv_high.^pipeline_delay;

M = ((1 - zinv_high.^(cic_filter_rate))./(1 - zinv_high)).^3;
M = M/abs(M(1));

L = M.*K.*Hpwm;
S = 1./(1 + L);
T = 1 - S;

%% Plot
% figure(1);clf;
% subplot(1,2,1);
% loglog(f,abs(S));
% plot_format('Frequency [MHz]','|L|','',10);
% grid on;
% subplot(1,2,2);
% loglog(f,unwrap(angle(L))*180/pi);
% plot_format('Frequency [MHz]','arg(L) [deg]','',10);
% grid on;

figure(1);clf;
subplot(1,3,1);
loglog(f,abs(L));
plot_format('Frequency [MHz]','|L|','',10);
grid on;
subplot(1,3,2);
loglog(f,abs(S));
plot_format('Frequency [MHz]','|S|','',10);
grid on;
% ylim([1e-3,Inf]);
subplot(1,3,3);
loglog(f,abs(T));
plot_format('Frequency [MHz]','|T|','',10);
grid on;
ylim([1e-6,10]);


%% LOCAL FUNCTIONS

function H = pwm_transfer_function(w)
R1 = 100;
C1 = 8.2e-9;
R2 = 0*2.2e3;
Resr = 2.478;
C2 = 47e-6;
C3 = 0e-6;
thorlabs_filter_freq = 2*pi*10e3;

ZC1 = 1./(1i*w*C1);
ZC2 = (1 + 1i*w*Resr*C2)./(1i*w*C2);
ZC2 = (1./ZC2 + 1i*w*C3).^-1;

Hlp = 1./(1 + 1i*w/(thorlabs_filter_freq));

H = ZC1.*ZC2./(R1.*ZC1 + (R1 + ZC1).*(R2 + ZC2));

H = H.*Hlp;

end

