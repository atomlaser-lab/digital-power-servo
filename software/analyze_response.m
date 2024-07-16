%
% Automatically measure transfer function
%
Npoints = 100e-3/servo.dt;
dV = 0.01;
Vmax = 0.25;
V = 0:dV:Vmax;
FigNum = 1600;

%% Measure steady-state response
check_app();
old_voltage = servo.pwm(1).value;
data = zeros(numel(V),1);
textprogressbar('RESET');
textprogressbar('Measuring AO0 response...');
for nn = 1:numel(V)
    textprogressbar(round(nn/numel(V)*100));
    servo.pwm(1).set(V(nn)).write;
    pause(250e-3);
    servo.getData(Npoints);
    data(nn) = mean(servo.data(:,1));
end
servo.pwm(1).set(old_voltage).write;
pause(500e-3);
textprogressbar('Done\n');

%% Plot steady-state response
figure(FigNum);clf;
plot(V,data,'o');
plot_format('Voltage [V]','Measurement [V]','IN1 response to AO0',10);
grid on;

G = diff(data)./dV;

%% Measure dynamic response
bias_voltage = servo.pwm(1).value;
jump_voltage = 10e-3;
acquisition_time = 200e-3;
servo.getVoltageStepResponse(acquisition_time/servo.dt,servo.output_switch(1).value,bias_voltage,jump_voltage);
data2 = servo.data;
t2 = servo.t;

%% Analyze dynamic response
nlf = nonlinfit(t2,data2(:,1));
nlf.ex = servo.t < (acquisition_time*0.2);
nlf.setFitFunc(@(y0,A,tau,x0,x) y0 + A*(1 - exp(-(x - x0)/tau)).*(x >= x0));
nlf.bounds2('y0',[-1,1,nlf.y(1)],'A',[-1,1,range(nlf.y)],'x0',[min(nlf.x),max(nlf.x),acquisition_time/4],...
    'tau',[0,1,1/10e3]);
nlf.fit;
figure(FigNum + 1);clf;
plot(nlf.x,nlf.y,'o');
hold on
plot(nlf.x,nlf.f(nlf.x),'--');

%% Compute gain values


%% LOCAL FUNCTIONS
function check_app
    app = PowerServoControl.get_running_app_instance();
    app.set_timer('off');
end

function update_app_display
    app = PowerServoControl.get_running_app_instance();
    app.updateDisplay;
end