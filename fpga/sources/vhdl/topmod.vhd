library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;
use work.AXI_Bus_Package.all;

--
-- Example top-level module for parsing simple AXI instructions
--
entity topmod is
    port (
        sysClk          :   in  std_logic;
        aresetn         :   in  std_logic;
        ext_i           :   in  std_logic_vector(7 downto 0);

        addr_i          :   in  unsigned(AXI_ADDR_WIDTH-1 downto 0);            --Address out
        writeData_i     :   in  std_logic_vector(AXI_DATA_WIDTH-1 downto 0);    --Data to write
        dataValid_i     :   in  std_logic_vector(1 downto 0);                   --Data valid out signal
        readData_o      :   out std_logic_vector(AXI_DATA_WIDTH-1 downto 0);    --Data to read
        resp_o          :   out std_logic_vector(1 downto 0);                   --Response in
        
        ext_o           :   out std_logic_vector(7 downto 0);
        led_o           :   out std_logic_vector(7 downto 0);
        pwm_o           :   out std_logic_vector(3 downto 0);
        
        adcClk          :   in  std_logic;
        adcClkx2        :   in  std_logic;
        adcData_i       :   in  std_logic_vector(31 downto 0);
       
        m_axis_tdata    :   out std_logic_vector(31 downto 0);
        m_axis_tvalid   :   out std_logic
      
    );
end topmod;


architecture Behavioural of topmod is

ATTRIBUTE X_INTERFACE_INFO : STRING;
ATTRIBUTE X_INTERFACE_INFO of m_axis_tdata: SIGNAL is "xilinx.com:interface:axis:1.0 m_axis TDATA";
ATTRIBUTE X_INTERFACE_INFO of m_axis_tvalid: SIGNAL is "xilinx.com:interface:axis:1.0 m_axis TVALID";
ATTRIBUTE X_INTERFACE_PARAMETER : STRING;
ATTRIBUTE X_INTERFACE_PARAMETER of m_axis_tdata: SIGNAL is "CLK_DOMAIN system_AXIS_Red_Pitaya_ADC_0_0_adc_clk,FREQ_HZ 125000000";
ATTRIBUTE X_INTERFACE_PARAMETER of m_axis_tvalid: SIGNAL is "CLK_DOMAIN system_AXIS_Red_Pitaya_ADC_0_0_adc_clk,FREQ_HZ 125000000";

component PIDController is
    port(
        --
        -- Clocking and reset
        --
        clk         :   in  std_logic;
        aresetn     :   in  std_logic;
        --
        -- Inputs
        --
        meas_i      :   in  t_meas;
        control_i   :   in  t_meas;
        valid_i     :   in  std_logic;
        --
        -- Parameters
        --
        enable_i    :   in  std_logic;
        polarity_i  :   in  std_logic;
        hold_i      :   in  std_logic;
        gains       :   in  t_param_reg;
        --
        -- Outputs
        --
        valid_o     :   out std_logic;
        data_o      :   out signed
    );
end component;

component DecimatingFilter is
    generic(
        NUM_INPUT_SIGNALS : natural :=  2
    );
    port(
        clk             :   in  std_logic;
        aresetn         :   in  std_logic;
        --
        -- Registers
        --
        filter_reg_i    :   in  t_param_reg;
        --
        -- Input and output data
        --
        data_i          :   in  t_adc_array(NUM_INPUT_SIGNALS - 1 downto 0);
        filtered_data_o :   out t_meas_array(NUM_INPUT_SIGNALS - 1 downto 0);
        valid_o         :   out std_logic_vector(NUM_INPUT_SIGNALS - 1 downto 0)
    );
end component;

component FIFOHandler is
    port(
        wr_clk      :   in  std_logic;
        rd_clk      :   in  std_logic;
        aresetn     :   in  std_logic;
        
        data_i      :   in  std_logic_vector(FIFO_WIDTH-1 downto 0);
        valid_i     :   in  std_logic;
        
        fifoReset   :   in  std_logic;
        bus_m       :   in  t_fifo_bus_master;
        bus_s       :   out t_fifo_bus_slave
    );
end component;

component SaveADCData is
    port(
        readClk     :   in  std_logic;          --Clock for reading data
        writeClk    :   in  std_logic;          --Clock for writing data
        aresetn     :   in  std_logic;          --Asynchronous reset
        
        data_i      :   in  std_logic_vector;   --Input data, maximum length of 32 bits
        valid_i     :   in  std_logic;          --High for one clock cycle when data_i is valid
        
        trigEdge    :   in  std_logic;          --'0' for falling edge, '1' for rising edge
        delay       :   in  unsigned;           --Acquisition delay
        numSamples  :   in  t_mem_addr;         --Number of samples to save
        trig_i      :   in  std_logic;          --Start trigger
        
        bus_m       :   in  t_mem_bus_master;   --Master memory bus
        bus_s       :   out t_mem_bus_slave     --Slave memory bus
    );
end component;

component PWM_Generator is
    port(
        --
        -- Clocking
        --
        clk         :   in  std_logic;
        clkx2       :   in  std_logic;
        aresetn     :   in  std_logic;
        --
        -- Input/outputs
        --
        data_i      :   in  t_pwm_array;
        valid_i     :   in  std_logic;
        pwm_o       :   out std_logic_vector   
    );
end component;

--
-- AXI communication signals
--
signal comState             :   t_status                        :=  idle;
signal bus_m                :   t_axi_bus_master                :=  INIT_AXI_BUS_MASTER;
signal bus_s                :   t_axi_bus_slave                 :=  INIT_AXI_BUS_SLAVE;
signal reset                :   std_logic;
--
-- Registers
--
signal topReg               :   t_param_reg;
signal triggers             :   t_param_reg;
signal outputReg            :   t_param_reg;
signal filterReg            :   t_param_reg;
-- PWM register
signal pwm_regs             :   t_param_reg_array(NUM_PIDS - 1 downto 0);
signal pwm_limit_regs       :   t_param_reg_array(NUM_PIDS - 1 downto 0);
-- FIFO register
signal fifoReg              :   t_param_reg;
-- PID registers
signal pid_regs             :   t_param_reg_array(1 downto 0);
signal pid_gain_regs        :   t_param_reg_array(1 downto 0);
-- DAC registers
signal dac_regs             :   t_param_reg_array(1 downto 0);
signal dac_limit_regs       :   t_param_reg_array(NUM_PIDS - 1 downto 0);
--
-- DAC signals
--
signal dac_o                :   t_dac_array(1 downto 0);
--
-- ADC signals
--
signal adc                  :   t_adc_array(1 downto 0);
signal filtered_data        :   t_meas_array(1 downto 0);
signal filter_valid         :   std_logic_vector(1 downto 0);
--
-- FIFO signals
--
constant NUM_FIFOS          :   natural :=  filtered_data'length;
type t_fifo_data_array is array(natural range <>) of std_logic_vector(FIFO_WIDTH - 1 downto 0);

signal fifoData             :   t_fifo_data_array(NUM_FIFOS - 1 downto 0);
signal fifoValid            :   std_logic_vector(NUM_FIFOS - 1 downto 0);
signal fifo_bus             :   t_fifo_bus_array(NUM_FIFOS - 1 downto 0)  :=  (others => INIT_FIFO_BUS);
signal enableFIFO           :   std_logic;
signal fifoReset            :   std_logic;
signal fifo_route           :   std_logic_vector(NUM_FIFOS - 1 downto 0);
signal fifo_actuator        :   t_fifo_data_array(NUM_FIFOS - 1 downto 0);

--
-- Memory signals
--
signal delay        :   unsigned(3 downto 0);
signal numSamples   :   t_mem_addr;
signal mem_bus      :   t_mem_bus;
signal mem_bus_m    :   t_mem_bus_master;
signal mem_bus_s    :   t_mem_bus_slave;
signal memTrig      :   std_logic;
--
-- PID signals
--
signal pid_control              :   t_meas_array(NUM_PIDS - 1 downto 0);
signal pid_enable               :   std_logic_vector(NUM_PIDS - 1 downto 0);
signal pid_polarity             :   std_logic_vector(NUM_PIDS - 1 downto 0);
signal pid_valid                :   std_logic_vector(NUM_PIDS - 1 downto 0);
signal pid_output               :   t_pid_output_array(NUM_PIDS - 1 downto 0); 
signal pid_output_switch        :   std_logic_vector(NUM_PIDS - 1 downto 0);
--
-- PWM signals
--
signal pwm_data, pwm_data_i     :   t_pwm_array(3 downto 0);
signal control_signal_o         :   t_pwm_exp_array(NUM_PIDs - 1 downto 0);
signal pwm_data_exp             :   t_pwm_exp_array(NUM_PIDS - 1 downto 0);
signal pwm_sum                  :   t_pwm_exp_array(NUM_PIDS - 1 downto 0);
signal pwm_limit                :   t_pwm_exp_array(NUM_PIDS - 1 downto 0);
signal pwm_max, pwm_min         :   t_pwm_exp_array(NUM_PIDS - 1 downto 0);
--
-- DAC signals
--
signal dac_data, dac_data_i     :   t_dac_array(1 downto 0);
signal dac_sum, dac_limit       :   t_dac_array(1 downto 0);
signal dac_max, dac_min         :   t_dac_array(1 downto 0);

begin

--
-- DAC Outputs
--
DAC_Gen: for I in 0 to 1 generate
    dac_data(I) <= signed(dac_regs(I)(t_dac'left downto 0));
end generate DAC_Gen;
dac_o(0) <= dac_limit(0);
dac_o(1) <= dac_limit(1);
m_axis_tdata <= std_logic_vector(dac_o(1)) & std_logic_vector(dac_o(0));
m_axis_tvalid <= '1';
--
-- PWM outputs
--
PWM_Gen: for I in 0 to NUM_PIDS - 1 generate
    pwm_data(I) <= unsigned(pwm_regs(I)(t_pwm'left downto 0));
    pwm_data_i(I) <= resize(unsigned(std_logic_vector(pwm_limit(I))),PWM_DATA_WIDTH);
end generate PWM_Gen;
pwm_data(2) <= (others => '0');
pwm_data_i(2) <= pwm_data(2);
pwm_data(3) <= (others => '0');
pwm_data_i(3) <= pwm_data(3);

PWM1: PWM_Generator
port map(
  clk     =>  adcClk,
  clkx2   =>  adcClkx2,
  aresetn =>  aresetn,
  data_i  =>  pwm_data_i,
  valid_i => '1',
  pwm_o   =>  pwm_o
);
-- 
-- Digital outputs
--
ext_o <= outputReg(7 downto 0);
led_o <= outputReg(15 downto 8);

--
-- Modulator/demodulator component
--
adc(0) <= resize(signed(adcData_i(15 downto 0)),t_adc'length);
adc(1) <= resize(signed(adcData_i(31 downto 16)),t_adc'length);

Main_Filter: DecimatingFilter
generic map(
    NUM_INPUT_SIGNALS   =>  filtered_data'length
)
port map(
    clk             =>  adcClk,
    aresetn         =>  aresetn,
    filter_reg_i    =>  filterReg,
    data_i          =>  adc,
    filtered_data_o =>  filtered_data,
    valid_o         =>  filter_valid
);

--
-- Apply feedback
--
pid_output_switch <= topReg(3 downto 2);
GEN_PID: for I in 0 to NUM_PIDS - 1 generate
    pid_enable(I) <= pid_regs(I)(0);
    pid_polarity(I) <= pid_regs(I)(1);
    pid_control(I) <= resize(signed(pid_regs(I)(31 downto 16)),t_meas'length);
    Controller: PIDController
    port map(
        clk         =>  adcClk,
        aresetn     =>  aresetn,
        meas_i      =>  filtered_data(I),
        control_i   =>  pid_control(I),
        valid_i     =>  filter_valid(I),
        enable_i    =>  pid_enable(I),
        polarity_i  =>  pid_polarity(I),
        hold_i      =>  '0',
        gains       =>  pid_gain_regs(I),
        valid_o     =>  pid_valid(I),
        data_o      =>  pid_output(I)
    );
end generate GEN_PID;


PWM_LIMIT_GEN: for I in 0 to NUM_PIDS - 1 generate
    -- Expand manual data to a signed 11 bit value
    pwm_data_exp(I) <= signed(std_logic_vector(resize(pwm_data(I),PWM_EXP_WIDTH)));
    -- Sum expanded manual data and control data
    pwm_sum(I) <= pwm_data_exp(I) + resize(pid_output(I),PWM_EXP_WIDTH) when pid_output_switch(I) = '0' else
                  pwm_data_exp(I);
    -- Parse limits, expand to 11 bits as signed values
    pwm_min(I) <= signed(resize(unsigned(pwm_limit_regs(I)(15 downto 0)),PWM_EXP_WIDTH));
    pwm_max(I) <= signed(resize(unsigned(pwm_limit_regs(I)(31 downto 16)),PWM_EXP_WIDTH));
    -- Limit the summed manual and control values to their max/min limits
    pwm_limit(I) <= pwm_sum(I) when pwm_sum(I) < pwm_max(I) and pwm_sum(I) > pwm_min(I) else
                    pwm_max(I) when pwm_sum(I) >= pwm_max(I) else
                    pwm_min(I) when pwm_sum(I) <= pwm_min(I);

end generate PWM_LIMIT_GEN;

DAC_LIMIT_GEN: for I in 0 to NUM_PIDS - 1 generate
    -- Sum manual data and control data
    dac_sum(I) <= dac_data(I) + resize(pid_output(I),t_dac'length) when pid_output_switch(I) = '1' else
                  dac_data(I);
    -- Parse limits
    dac_min(I) <= resize(signed(dac_limit_regs(I)(15 downto 0)),t_dac'length);
    dac_max(I) <= resize(signed(dac_limit_regs(I)(31 downto 16)),t_dac'length);
    -- Limit the summed manual data and control values to max/min limits
    dac_limit(I) <= dac_sum(I) when dac_sum(I) < dac_max(I) and dac_sum(I) > dac_min(I) else
                    dac_max(I) when dac_sum(I) >= dac_max(I) else
                    dac_min(I) when dac_sum(I) <= dac_min(I);
end generate DAC_LIMIT_GEN;

--
-- Collect demodulated data at lower sampling rate in FIFO buffers
-- to be read out continuously by CPU
--
enableFIFO <= fifoReg(0);
fifoReset <= fifoReg(1);
fifo_route <= topReg(1 downto 0);
FIFO_GEN: for I in 0 to NUM_FIFOS - 1 generate
    fifo_actuator(I) <= std_logic_vector(resize(pwm_limit(I),FIFO_WIDTH)) when pid_output_switch(I) = '0' else
                       std_logic_vector(resize(dac_limit(I),FIFO_WIDTH));
    fifoData(I) <= std_logic_vector(resize(filtered_data(I),FIFO_WIDTH)) when fifo_route(I) = '0' else fifo_actuator(I);
    fifoValid(I) <= ((filter_valid(I) and (not(fifo_route(I)) or not(pid_enable(I)))) or (pid_valid(I) and fifo_route(I) and pid_enable(I))) and enableFIFO;
    PhaseMeas_FIFO_NORMAL_X: FIFOHandler
    port map(
        wr_clk      =>  adcClk,
        rd_clk      =>  sysClk,
        aresetn     =>  aresetn,
        data_i      =>  fifoData(I),
        valid_i     =>  fifoValid(I),
        fifoReset   =>  fifoReset,
        bus_m       =>  fifo_bus(I).m,
        bus_s       =>  fifo_bus(I).s
  );
end generate FIFO_GEN;
--
-- Save ADC data for debugging purposes
--
delay     <= (others => '0');
memTrig   <= triggers(0);
SaveData: SaveADCData
port map(
    readClk     =>  sysClk,
    writeClk    =>  adcClk,
    aresetn     =>  aresetn,
    data_i      =>  adcData_i,
    valid_i     =>  '1',
    trigEdge    =>  '1',
    delay       =>  delay,
    numSamples  =>  numSamples,
    trig_i      =>  memTrig,
    bus_m       =>  mem_bus.m,
    bus_s       =>  mem_bus.s
);
--
-- AXI communication routing - connects bus objects to std_logic signals
--
bus_m.addr <= addr_i;
bus_m.valid <= dataValid_i;
bus_m.data <= writeData_i;
readData_o <= bus_s.data;
resp_o <= bus_s.resp;

-- Assigning the ouput control signal to pwm output values


Parse: process(sysClk,aresetn) is
begin
    if aresetn = '0' then
        comState <= idle;
        reset <= '0';
        bus_s <= INIT_AXI_BUS_SLAVE;
        triggers <= (others => '0');
        outputReg <= (others => '0');
        filterReg <= X"0000000a";
        pwm_regs <= (others => (others => '0'));
        dac_regs <= (others => (others => '0'));
        
        for I in 0 to pid_regs'length - 1 loop
            pid_regs(I) <= (others => '0');
        end loop;

        for I in 0 to pid_gain_regs'length - 1 loop
            pid_gain_regs(I) <= (others => '0');
        end loop;

        for I in 0 to pwm_limit_regs'length - 1 loop
            pwm_limit_regs(I) <= (others => '0');
        end loop;

        for I in 0 to dac_limit_regs'length - 1 loop
            dac_limit_regs(I) <= (others => '0');
        end loop;
        --
        -- FIFO registers
        --
        fifoReg <= (others => '0');
        for I in 0 to NUM_FIFOS - 1 loop
            fifo_bus(I).m.status <= idle;
        end loop;
        --
        -- Memory signals
        --
        numSamples <= to_unsigned(4000,numSamples'length);
        mem_bus.m <= INIT_MEM_BUS_MASTER; 

    elsif rising_edge(sysClk) then
        FSM: case(comState) is
            when idle =>
                triggers <= (others => '0');
                reset <= '0';
                bus_s.resp <= "00";
                mem_bus.m.reset <= '0';
                if bus_m.valid(0) = '1' then
                    comState <= processing;
                end if;

            when processing =>
                AddrCase: case(bus_m.addr(31 downto 24)) is
                    --
                    -- Parameter parsing
                    --
                    when X"00" =>
                        ParamCase: case(bus_m.addr(23 downto 0)) is
                            when X"000000" => rw(bus_m,bus_s,comState,triggers);
                            when X"000004" => rw(bus_m,bus_s,comState,topReg);
                            when X"000008" => rw(bus_m,bus_s,comState,outputReg);
                            when X"00000C" => rw(bus_m,bus_s,comState,filterReg);
                            when X"000010" => readOnly(bus_m,bus_s,comState,adcData_i);
                            when X"000014" => readOnly(bus_m,bus_s,comState,ext_i);
                            --
                            -- PID registers
                            --
                            when X"000100" => rw(bus_m,bus_s,comState,pid_regs(0));
                            when X"000104" => rw(bus_m,bus_s,comState,pid_regs(1));
                            when X"000108" => rw(bus_m,bus_s,comState,pid_gain_regs(0));
                            when X"00010C" => rw(bus_m,bus_s,comState,pid_gain_regs(1));
                            --
                            -- PWM registers
                            --
                            when X"000200" => rw(bus_m,bus_s,comState,pwm_regs(0));
                            when X"000204" => rw(bus_m,bus_s,comState,pwm_regs(1));
                            when X"000210" => rw(bus_m,bus_s,comState,pwm_limit_regs(0));
                            when X"000214" => rw(bus_m,bus_s,comState,pwm_limit_regs(1));
                            -- 
                            -- DAC registers
                            --
                            when X"000300" => rw(bus_m,bus_s,comState,dac_regs(0));
                            when X"000304" => rw(bus_m,bus_s,comState,dac_regs(1));
                            when X"000310" => rw(bus_m,bus_s,comState,dac_limit_regs(0));
                            when X"000314" => rw(bus_m,bus_s,comState,dac_limit_regs(1));
                            --
                            -- FIFO control and data retrieval
                            --
                            when X"000084" => rw(bus_m,bus_s,comState,fifoReg);
                            when X"000088" => fifoRead(bus_m,bus_s,comState,fifo_bus(0).m,fifo_bus(0).s);
                            when X"00008C" => fifoRead(bus_m,bus_s,comState,fifo_bus(1).m,fifo_bus(1).s);
                            --
                            -- Memory signals
                            --
                            when X"100000" => rw(bus_m,bus_s,comState,numSamples);
                            when X"100004" =>
                                bus_s.resp <= "01";
                                comState <= finishing;
                                mem_bus.m.reset <= '1';
                           
                            when others => 
                                comState <= finishing;
                                bus_s.resp <= "11";
                        end case;

                      --
                    -- Memory reading of normal memory
                    --
                    when X"01" =>  
                      if bus_m.valid(1) = '0' then
                          bus_s.resp <= "11";
                          comState <= finishing;
                          mem_bus.m.trig <= '0';
                          mem_bus.m.status <= idle;
                      elsif mem_bus.s.valid = '1' then
                          bus_s.data <= mem_bus.s.data;
                          comState <= finishing;
                          bus_s.resp <= "01";
                          mem_bus.m.status <= idle;
                          mem_bus.m.trig <= '0';
                      elsif mem_bus.s.status = idle then
                          mem_bus.m.addr <= bus_m.addr(MEM_ADDR_WIDTH+1 downto 2);
                          mem_bus.m.status <= waiting;
                          mem_bus.m.trig <= '1';
                      else
                          mem_bus.m.trig <= '0';
                      end if;
                    
                    when others => 
                        comState <= finishing;
                        bus_s.resp <= "11";
                end case;
            when finishing =>
                comState <= idle;

            when others => comState <= idle;
        end case;
    end if;
end process;

end architecture Behavioural;