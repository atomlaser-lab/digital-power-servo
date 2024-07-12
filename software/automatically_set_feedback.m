%
% Automatically measure transfer function
%
Npoints = 1e3;
dV = 0.01;
Vmax = 0.25;
V = 0:dV:Vmax;
FigNum = 1500;

%% Measure linear response
check_app();
old_voltage = servo.pwm(1).value;
data = zeros(numel(V),1);
textprogressbar('RESET');
textprogressbar('Measuring AO0 response...');
for nn = 1:numel(V)
    textprogressbar(round(nn/numel(V)*100));
    servo.pwm(1).set(V(nn)).write;
    pause(100e-3);
    servo.getData(Npoints);
    data(nn) = mean(servo.data(:,1));
end
servo.pwm(1).set(old_voltage).write;
textprogressbar('Done\n');

%% Plot linear response
figure(FigNum);clf;
plot(V,data,'o');
plot_format('Voltage [V]','Measurement [V]','IN1 response to AO0',10);
grid on;
% lf = linfit(V,data);
% lf.setFitFunc('poly',4);
% lf.fit;
% lf.plot('plotresiduals',0);


%% LOCAL FUNCTIONS
function check_app
    app = PowerServoControl.get_running_app_instance();
    app.set_timer('off');
end

function update_app_display
    app = PowerServoControl.get_running_app_instance();
    app.updateDisplay;
end