classdef PowerServoControl < handle
    properties
        jumpers
        t
        data
        auto_retry
    end
    
    properties(SetAccess = immutable)
        conn                    %Instance of CONNECTIONCLIENT used for communication with socket server
        %
        % All of these are DEVICEPARAMETER objects
        %
        triggers                %Triggers -- currently unused
        log2_rate               %Log2(CIC filter rate)
        cic_shift               %Log2(Additional digital gain after filter)
        numSamples              %Number of samples to collect from recording raw ADC signals
        pwm                     %Array of 2 PWM outputs
        pwm_upper_limits        %Array of 2 PWM upper limits
        pwm_lower_limits        %Array of 2 PWM lower limits
        dac                     %Array of 2 DAC outputs
        dac_upper_limits        %Array of 2 DAC upper limits
        dac_lower_limits        %Array of 2 DAC lower limits
        pids                    %PID control
        output_switch           %Switch controlling PID output: 0 (PWM), 1 (DAC)
        fifo_route              %Array of 2 FIFO routing options
    end
    
    properties(SetAccess = protected)
        % R/W registers
        topReg                  %Top-level register
        trigReg                 %Register for software trigger signals
        filterReg               %Register for CIC filter control
        numSamplesReg           %Register for storing number of samples of ADC data to fetch
        pwmRegs                 %Registers for PWM signals
        dacRegs                 %Registers for DAC signals
        auxReg                  %Auxiliary register
        pidControlRegs          %Registers for control
        pidGainRegs             %Registers for gain values
        pwmLimitRegs            %Registers for limiting PWM outputs
        dacLimitRegs            %Registers for limiting DAC outputs
    end
    
    properties(Constant)
        CLK = 125e6;
        DEFAULT_HOST = 'rp-f06a54.local';
        DAC_WIDTH = 14;
        ADC_WIDTH = 14;
        PARAM_WIDTH = 32;
        PWM_WIDTH = 10;
        NUM_PID = 2;
        NUM_PWM = 2;
        NUM_MEAS = 2;
        NUM_DAC = 2;
        %
        % Conversion values going from integer values to volts
        %
        CONV_ADC_LV = 1.1851/2^(PowerServoControl.ADC_WIDTH - 1);
        CONV_ADC_HV = 29.3570/2^(PowerServoControl.ADC_WIDTH - 1);
        CONV_DAC = 1/(2^(PowerServoControl.DAC_WIDTH - 1) - 1);
        CONV_PWM = 1.6/(2^PowerServoControl.PWM_WIDTH - 1);
        %
        % Output options
        %
        PID_OUTPUT_OPTIONS = {'AO0','AO1','OUT1','OUT2'};
        FIFO_ROUTE_OPTIONS = {'IN1','IN2','AO0','AO1','OUT1','OUT2'};
    end
    
    methods
        function self = PowerServoControl(varargin)
            %POWERSERVOCONTROL Creates an instance of a POWERSERVOCONTROL object.  Sets
            %up the registers and parameters as instances of the correct
            %classes with the necessary
            %addressses/registers/limits/functions
            %
            %   SELF = POWERSERVOCONTROL() creates an instance with default host
            %   and port
            %
            %   SELF = POWERSERVOCONTROL(HOST) creates an instance with socket
            %   server host address HOST
            
            if numel(varargin) == 1
                self.conn = ConnectionClient(varargin{1});
            else
                self.conn = ConnectionClient(self.DEFAULT_HOST);
            end
            %
            % Set jumper values
            %
            self.jumpers = 'lv';
            %
            % Registers
            %
            self.trigReg = DeviceRegister('0',self.conn);
            self.topReg = DeviceRegister('4',self.conn);
            self.filterReg = DeviceRegister('C',self.conn);
            self.pwmRegs = DeviceRegister.empty;
            self.pwmLimitRegs = DeviceRegister.empty;
            for nn = 1:self.NUM_PWM
                self.pwmRegs(nn) = DeviceRegister(hex2dec('200') + (nn - 1)*4,self.conn);
                self.pwmLimitRegs(nn,1) = DeviceRegister(hex2dec('210') + (nn - 1)*4,self.conn);
            end
            self.dacRegs = DeviceRegister.empty;
            self.dacLimitRegs = DeviceRegister.empty;
            for nn = 1:self.NUM_DAC
                self.dacRegs(nn) = DeviceRegister(hex2dec('300') + (nn - 1)*4,self.conn);
                self.dacLimitRegs(nn,1) = DeviceRegister(hex2dec('310') + (nn - 1)*4,self.conn);
            end
            self.numSamplesReg = DeviceRegister('100000',self.conn);
            %
            % PID registers
            %
            self.pidControlRegs = DeviceRegister.empty;
            self.pidGainRegs = DeviceRegister.empty;           
            for nn = 1:self.NUM_PID
                self.pidControlRegs(nn,1) = DeviceRegister(hex2dec('100') + (nn - 1)*4,self.conn);
                self.pidGainRegs(nn,1) = DeviceRegister(hex2dec('108') + (nn - 1)*4,self.conn);

            end
            %
            % Auxiliary register for all and sundry
            %
            self.auxReg = DeviceRegister('100004',self.conn);
            %
            % Filter settings
            %
            self.log2_rate = DeviceParameter([0,3],self.filterReg,'uint32')...
                .setLimits('lower',2,'upper',13);
            self.cic_shift = DeviceParameter([4,11],self.filterReg,'int8')...
                .setLimits('lower',-100,'upper',100);
            %
            % PWM settings
            %
            self.pwm = DeviceParameter.empty;
            self.pwm_upper_limits = DeviceParameter.empty;
            self.pwm_lower_limits = DeviceParameter.empty;
            for nn = 1:self.NUM_PWM
                self.pwm(nn) = DeviceParameter([0,self.PWM_WIDTH - 1],self.pwmRegs(nn))...
                    .setLimits('lower',0,'upper',1.62)...
                    .setFunctions('to',@(x) x/self.CONV_PWM,'from',@(x) x*self.CONV_PWM);
                self.pwm_lower_limits(nn) = DeviceParameter([0,15],self.pwmLimitRegs(nn))...
                    .setLimits('lower',0,'upper',1.62)...
                    .setFunctions('to',@(x) x/self.CONV_PWM,'from',@(x) x*self.CONV_PWM);
                self.pwm_upper_limits(nn) = DeviceParameter([16,31],self.pwmLimitRegs(nn))...
                    .setLimits('lower',0,'upper',1.62)...
                    .setFunctions('to',@(x) x/self.CONV_PWM,'from',@(x) x*self.CONV_PWM);
            end
            %
            % DAC settings
            %
            self.dac = DeviceParameter.empty;
            self.dac_upper_limits = DeviceParameter.empty;
            self.dac_lower_limits = DeviceParameter.empty;
            for nn = 1:self.NUM_DAC
                self.dac(nn) = DeviceParameter([0,15],self.dacRegs(nn),'int16')...
                    .setLimits('lower',-1,'upper',1)...
                    .setFunctions('to',@(x) x/self.CONV_DAC,'from',@(x) x*self.CONV_DAC);
                self.dac_lower_limits(nn) = DeviceParameter([0,15],self.dacLimitRegs(nn),'int16')...
                    .setLimits('lower',-1,'upper',1)...
                    .setFunctions('to',@(x) x/self.CONV_DAC,'from',@(x) x*self.CONV_DAC);
                self.dac_upper_limits(nn) = DeviceParameter([16,31],self.dacLimitRegs(nn),'int16')...
                    .setLimits('lower',-1,'upper',1)...
                    .setFunctions('to',@(x) x/self.CONV_DAC,'from',@(x) x*self.CONV_DAC);
            end
            %
            % Number of samples for reading raw ADC data
            %
            self.numSamples = DeviceParameter([0,11],self.numSamplesReg,'uint32')...
                .setLimits('lower',0,'upper',2^12);
            %
            % PID settings
            %
            self.output_switch = DeviceParameter.empty;
            self.pids = PowerServoPID.empty;
            for nn = 1:self.NUM_PID
                self.pids(nn,1) = PowerServoPID(self,self.pidControlRegs(nn),self.pidGainRegs(nn));
                self.output_switch(nn,1) = DeviceParameter([8,9] + 2*(nn - 1),self.topReg).setLimits('lower',0,'upper',3)...
                    .setFunctions('to',@(x) self.convert_pid_table(x),'from',@(x) self.convert_pid_table(x));
            end
            %
            % FIFO routing
            %
            self.fifo_route = DeviceParameter.empty;
            for nn = 1:self.NUM_MEAS
                self.fifo_route(nn) = DeviceParameter((nn - 1)*4 + [0,3],self.topReg,'uint32')...
                    .setLimits('lower',0,'upper',5)...
                    .setFunctions('to',@(x) self.convert_fifo_table(x),'from',@(x) self.convert_fifo_table(x));
            end
        end
        
        function self = setDefaults(self,varargin)
            %SETDEFAULTS Sets parameter values to their defaults
            %
            %   SELF = SETDEFAULTS(SELF) sets default values for SELF
            self.pwm.set([0,0]);
            self.log2_rate.set(13);
            self.cic_shift.set(0);
            self.numSamples.set(4000);
            self.output_switch(1).set('AO0');
            self.output_switch(2).set('AO1');
            self.dac.set([0,0]);
            self.pwm_lower_limits.set([0,0]);
            self.pwm_upper_limits.set([0.25,0.25]);
            self.dac_lower_limits.set([0,0]);
            self.dac_upper_limits.set([0.25,0.25]);
            self.pids.setDefaults;
            for nn = 1:numel(self.fifo_route)
                self.fifo_route(nn).set(sprintf('IN%d',nn));
            end

            self.auto_retry = true;
        end

        function r = dt(self)
            %DT Returns the current sampling time based on the filter
            %settings
            %
            %   R = DT(SELF) returns sampling time R for DEVICECONTROL object
            %   SELF
            r = 2^(self.log2_rate.value)/self.CLK;
        end
        
        function self = check(self)

        end

%         function self = set_output_devices(self,devices)
%             %SET_OUTPUT_DEVICE Sets the PID output devices to either PWM or
%             %DAC
%             if iscell(devices) && numel(devices) == 2
%                 for nn = 1:self.NUM_PID
%                     if strcmpi(devices{nn},'pwm')
%                         self.output_switch(nn).set(0);
%             if strcmpi(devices,'pwm')
% 
%         end
        
        function self = upload(self)
            %UPLOAD Uploads register values to the device
            %
            %   SELF = UPLOAD(SELF) uploads register values associated with
            %   object SELF
            
            %
            % Check parameters first
            %
            self.check;
            %
            % Get all write data
            %
            p = properties(self);
            d = [];
            for nn = 1:numel(p)
                if isa(self.(p{nn}),'DeviceRegister')
                    R = self.(p{nn});
                    if numel(R) == 1
                        if ~R.read_only
                            d = [d;self.(p{nn}).getWriteData]; %#ok<*AGROW>
                        end
                    else
                        for row = 1:size(R,1)
                            for col = 1:size(R,2)
                                if ~R(row,col).read_only
                                    d = [d;R(row,col).getWriteData];
                                end
                            end
                        end
                    end
                end
            end

            d = d';
            d = d(:);
            %
            % Write every register using the same connection
            %
            self.conn.write(d,'mode','write');
        end
        
        function self = fetch(self)
            %FETCH Retrieves parameter values from the device
            %
            %   SELF = FETCH(SELF) retrieves values and stores them in
            %   object SELF
            
            %
            % Get addresses to read from for each register and get data
            % from device
            %
            p = properties(self);
            pread = {};
            Rread = DeviceRegister.empty;
            d = [];
            for nn = 1:numel(p)
                if isa(self.(p{nn}),'DeviceRegister')
                    R = self.(p{nn});
                    if numel(R) == 1
                        d = [d;R.getReadData];
                        Rread(end + 1) = R;
                        pread{end + 1} = p{nn};
                    else
                        pread{end + 1} = p{nn};
                        for row = 1:size(R,1)
                            for col = 1:size(R,2)
                                d = [d;R(row,col).getReadData];
                                Rread(end + 1) = R(row,col);
                            end
                        end
                    end
                    
                end
            end
            self.conn.write(d,'mode','read');
            value = self.conn.recvMessage;
            %
            % Parse the received data in the same order as the addresses
            % were written
            %
            for nn = 1:numel(value)
                Rread(nn).value = value(nn);
            end
            %
            % Read parameters from registers
            %
            p = properties(self);
            for nn = 1:numel(p)
                if isa(self.(p{nn}),'DeviceParameter') || isa(self.(p{nn}),'PowerServoSubModule')
                    self.(p{nn}).get;
                end
            end
        end

        function self = memoryReset(self)
            %MEMORYRESET Resets the two block memories
            self.auxReg.addr = '100004';
            self.auxReg.write;
        end
        
        function r = convert2volts(self,x)
            if strcmpi(self.jumpers,'hv')
                c = self.CONV_ADC_HV;
            elseif strcmpi(self.jumpers,'lv')
                c = self.CONV_ADC_LV;
            end
            r = x*c*2^(self.cic_shift.value);
        end
        
        function r = convert2int(self,x)
            if strcmpi(self.jumpers,'hv')
                c = self.CONV_ADC_HV;
            elseif strcmpi(self.jumpers,'lv')
                c = self.CONV_ADC_LV;
            end
            r = x/c/2^(self.cic_shift.value);
        end


        function self = getData(self,numSamples,saveType)
            %GETDATA Fetches data from the device
            %
            %   SELF = GETDATA(NUMSAMPLES) Acquires NUMSAMPLES of data
            %
            %   SELF = GETDATA(__,SAVETYPE) uses SAVETYPE for saving data.  For advanced
            %   users only: see the readme
            numSamples = round(numSamples);
            if nargin < 3
                saveType = 1;
            end
            write_arg = {'./saveData','-n',sprintf('%d',numSamples),'-t',sprintf('%d',saveType),'-s',sprintf('%d',PowerServoControl.NUM_MEAS)};
            if self.auto_retry
                for jj = 1:10
                    try
                        self.conn.write(0,'mode','command','cmd',write_arg,'return_mode','file');
                        raw = typecast(self.conn.recvMessage,'uint8');
                        d = self.convertData(raw);
                        self.data = d;
                        self.t = self.dt()*(0:(numSamples-1));
                        break;
                    catch e
                        if jj == 10
                            rethrow(e);
                        end
                    end
                end
            else
                self.conn.write(0,'mode','command','cmd',write_arg,'return_mode','file');
                raw = typecast(self.conn.recvMessage,'uint8');
                d = self.convertData(raw);
                self.data = d;
                self.t = self.dt()*(0:(numSamples-1));
            end

            for nn = 1:self.NUM_MEAS
                if strfind(self.fifo_route(nn).value,'IN')
                    self.data(:,nn) = self.convert2volts(self.data(:,nn));
                elseif strfind(self.fifo_route(nn).value,'OUT')
                    self.data(:,nn) = self.data(:,nn)*self.CONV_DAC;
                else
                    self.data(:,nn) = self.data(:,nn)*self.CONV_PWM;
                end
            end
        end

        function self = getRAM(self,numSamples)
            %GETRAM Fetches recorded in block memory from the device
            %
            %   SELF = GETRAM(SELF) Retrieves current number of recorded
            %   samples from the device SELF
            %
            %   SELF = GETRAM(SELF,N) Retrieves N samples from device
            numSamples = round(numSamples);
            self.numSamples.set(numSamples).write;
            self.trigReg.set(1,[0,0]).write;
            self.trigReg.set(0,[0,0]);
            
            self.conn.write(0,'mode','command','cmd',...
                {'./fetchRAM',sprintf('%d',numSamples)},...
                'return_mode','file');
            raw = typecast(self.conn.recvMessage,'uint8');
            if strcmpi(self.jumpers,'hv')
                c = self.CONV_ADC_HV;
            elseif strcmpi(self.jumpers,'lv')
                c = self.CONV_ADC_LV;
            end
            d = self.convertADCData(raw,c);
            self.data = d;
            dt = self.CLK^-1;
            self.t = dt*(0:(size(self.data,1)-1));
        end
        
        function disp(self)
            strwidth = 20;
            fprintf(1,'PowerServoControl object with properties:\n');
            fprintf(1,'\t Registers\n');
            p = properties(self);
            for nn = 1:numel(p)
                if isa(self.(p{nn}),'DeviceRegister')
                    self.(p{nn}).print(p{nn},strwidth);
                end
            end            
            fprintf(1,'\t ----------------------------------\n');
            fprintf(1,'\t Parameters\n')
            self.log2_rate.print('Log2 Rate',strwidth,'%.0f');
            self.cic_shift.print('CIC shift',strwidth,'%.0f');                     
            for nn = 1:numel(self.pwm)
                self.pwm(nn).print(sprintf('PWM %d',nn),strwidth,'%.3f','V');
                self.pwm_lower_limits(nn).print(sprintf('PWM %d lower limit',nn),strwidth,'%.3f','V');
                self.pwm_upper_limits(nn).print(sprintf('PWM %d upper limit',nn),strwidth,'%.3f','V');
            end
            for nn = 1:numel(self.dac)
                self.dac(nn).print(sprintf('DAC %d',nn),strwidth,'%.3f','V');
                self.dac_lower_limits(nn).print(sprintf('DAC %d lower limit',nn),strwidth,'%.3f','V');
                self.dac_upper_limits(nn).print(sprintf('DAC %d upper limit',nn),strwidth,'%.3f','V');
            end
            
            for nn = 1:numel(self.fifo_route)
                self.fifo_route(nn).print(sprintf('FIFO Route %d',nn),strwidth,'%s');
            end
            for nn = 1:numel(self.output_switch)
                self.output_switch(nn).print(sprintf('Output switch %d',nn),strwidth,'%s');
            end
            for nn = 1:self.NUM_PID
                fprintf(1,'\t ----------------------------------\n');
                fprintf(1,'\t PID %d Parameters\n',nn);
                self.pids(nn).print(strwidth);
            end
        end
        
        function s = struct(self)
            %STRUCT Returns a structure representing the data
            %
            %   S = STRUCT(SLEF) Returns structure S from current object
            %   SELF
            s.conn = self.conn.struct;
            s.t = self.t;
            s.data = self.data;
            s.jumpers = self.jumpers;
            
            p = properties(self);
            for nn = 1:numel(p)
                if isa(self.(p{nn}),'DeviceParameter') || isa(self.(p{nn}),'PowerServoSubModule')
                    s.(p{nn}) = self.(p{nn}).struct;
                end
            end
        end
        
        function s = saveobj(self)
            %SAVEOBJ Returns a structure used for saving data
            %
            %   S = SAVEOBJ(SELF) Returns structure S used for saving data
            %   representing object SELF
            s = self.struct;
        end
        
        function self = loadstruct(self,s)
            %LOADSTRUCT Loads a struct and copies properties to object
            %
            %   SELF = LOADSTRUCT(SELF,S) Copies properties from structure
            %   S to SELF
            self.t = s.t;
            self.data = s.data;
            self.jumpers = s.jumpers;
            p = properties(self);
            for nn = 1:numel(p)
                if isfield(s,p{nn})
                    if isa(self.(p{nn}),'DeviceParameter') || isa(self.(p{nn}),'PowerServoSubModule')
                        try
                            self.(p{nn}).loadstruct(s.(p{nn}));
                        catch
                            
                        end
                    end
                end
            end
        end
        
    end

    methods(Static)
        function self = loadobj(s)
            %LOADOBJ Creates a DEVIECCONTROL object using input structure
            %
            %   SELF = LOADOBJ(S) uses structure S to create new
            %   DEVICECONTROL object SELF
            self = PowerServoControl(s.conn.host,s.conn.port);
            self.setDefaults;
            self.loadstruct(s);
        end
        
        
        function d = convertData(raw)
            raw = raw(:);
            Nraw = numel(raw);
            numStreams = PowerServoControl.NUM_MEAS;
            d = zeros(Nraw/(numStreams*4),numStreams,'int32');
            
            raw = reshape(raw,4*numStreams,Nraw/(4*numStreams));
            for nn = 1:numStreams
                d(:,nn) = typecast(uint8(reshape(raw((nn-1)*4 + (1:4),:),4*size(d,1),1)),'int32');
            end
            d = double(d);
        end

        function v = convertADCData(raw,c)
            %CONVERTADCDATA Converts raw ADC data into proper int16/double format
            %
            %   V = CONVERTADCDATA(RAW) Unpacks raw data from uint8 values to
            %   a pair of double values for each measurement
            %
            %   V = CONVERTADCDATA(RAW,C) uses conversion factor C in the
            %   conversion
            
            if nargin < 2
                c = 1;
            end
            
            Nraw = numel(raw);
            d = zeros(Nraw/4,2,'int16');
            
            mm = 1;
            for nn = 1:4:Nraw
                d(mm,1) = typecast(uint8(raw(nn + (0:1))),'int16');
                d(mm,2) = typecast(uint8(raw(nn + (2:3))),'int16');
                mm = mm + 1;
            end
            
            v = double(d)*c;
        end

        function r = convert_fifo_table(x)
            if ischar(x) || isstring(x)
                r = find(strcmpi(x,PowerServoControl.FIFO_ROUTE_OPTIONS)) - 1;
            elseif isnumeric(x)
                r = PowerServoControl.FIFO_ROUTE_OPTIONS{x + 1};
            end
        end

        function r = convert_pid_table(x)
            if ischar(x) || isstring(x)
                r = find(strcmpi(x,PowerServoControl.PID_OUTPUT_OPTIONS)) - 1;
            elseif isnumeric(x)
                r = PowerServoControl.PID_OUTPUT_OPTIONS{x + 1};
            end
        end

        function app = get_running_app_instance
            h = findall(groot,'type','figure');
            for nn = 1:numel(h)
                if strcmpi(h(nn).Name,'Power Servo Control')
                    app = h(nn).RunningAppInstance;
                    break;
                end
            end
        end

    end
    
end

