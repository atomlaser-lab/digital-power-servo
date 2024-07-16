classdef PowerServoPID < PowerServoSubModule
    %LASERSERVOPID Defines a class for handling the PID modules in the
    %laser servo
    
    properties(SetAccess = immutable)
        Kp              %Proportional gain value
        Ki              %Integral gain value
        Kd              %Derivative gain value
        divisor         %Overall divisor for gain values to convert to fractions
        polarity        %Polarity of PID module
        enable          %Enable/disable PID module
        control         %Control/set-point of the module
    end

    properties(Constant)
        PID_ACCUM_WIDTH = 32;   %This sets the max "divisor" value
    end
    
    methods
        function self = PowerServoPID(parent,control_reg,gain_reg)
            %PowerServoPID Creates an instance of the class
            %
            %   SELF = PowerServoPID(PARENT,REGS) Creates instance SELF
            %   with parent object PARENT and associated with registers
            %   REGS
            
            self.parent = parent;
            
            self.enable = DeviceParameter([0,0],control_reg)...
                .setLimits('lower',0,'upper',1);
            self.polarity = DeviceParameter([1,1],control_reg)...
                .setLimits('lower',0,'upper',1);
            self.control = DeviceParameter([16,31],control_reg,'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.Kp = DeviceParameter([0,7],gain_reg)...
                .setLimits('lower',0,'upper',2^8-1);
            self.Ki = DeviceParameter([8,15],gain_reg)...
                .setLimits('lower',0,'upper',2^8-1);
            self.Kd = DeviceParameter([16,23],gain_reg)...
                .setLimits('lower',0,'upper',2^8-1);
            self.divisor = DeviceParameter([24,31],gain_reg)...
                .setLimits('lower',0,'upper',self.PID_ACCUM_WIDTH);
        end
        
        function self = setDefaults(self)
            %SETDEFAULTS Sets the default values for the module
            %
            %   SELF = SETDEFAULTS(SELF) sets the default values of object
            %   SELF
            
            if numel(self) > 1
                for nn = 1:numel(self)
                    self(nn).setDefaults;
                end
            else
                self.enable.set(0);
                self.polarity.set(0);
                self.control.set(0);
                self.Kp.set(0);
                self.Ki.set(0);
                self.Kd.set(0);
                self.divisor.set(8);
            end
        end
        
        function [Kp,Ki,Kd] = calculateRealGains(self)
            %CALCULATEREALGAINS Calculates the "real",
            %continuous-controller equivalent gains
            %
            %   [Kp,Ki,Kd] = CALCULATEREALGAINS(SELF) Calculates the real
            %   gains Kp, Ki, and Kd using the set DIVISOR value and the
            %   parent object's sampling interval
            
            Kp = self.Kp.value*2^(-self.divisor.value);
            Ki = self.Ki.value*2^(-self.divisor.value)/self.parent.dt();
            Kd = self.Kd.value*2^(-self.divisor.value)*self.parent.dt();
        end
        
        function ss = print(self,width)
            %PRINT Prints a string representing the object
            %
            %   S = PRINT(SELF,WIDTH) returns a string S representing the
            %   object SELF with label width WIDTH.  If S is not requested,
            %   prints it to the command line
            s{1} = self.enable.print('Enable',width,'%d');
            s{2} = self.polarity.print('Polarity',width,'%d');
            s{3} = self.control.print('Control',width,'%.3f','V');
            s{4} = self.Kp.print('Kp',width,'%d');
            s{5} = self.Ki.print('Ki',width,'%d');
            s{6} = self.Kd.print('Kd',width,'%d');
            s{7} = self.divisor.print('Divisor',width,'%d');
            
            ss = '';
            for nn = 1:numel(s)
                ss = [ss,s{nn}]; %#ok<*AGROW>
            end
            if nargout == 0
                fprintf(1,ss);
            end
        end
        
        function disp(self)
            %DISP Displays the object properties
            disp('PowerServoPID object with properties:');
            disp(self.print(25));
        end
        
    end
    
end