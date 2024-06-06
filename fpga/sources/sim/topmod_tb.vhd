library IEEE;
library work;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;
use work.AXI_Bus_Package.all;

entity topmod_tb is
--  Port ( );
end topmod_tb;

architecture Behavioral of topmod_tb is

component topmod is
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
end component;

component AXI_Tester is
    port (
        --
        -- Clocking and reset
        --
        clk         :   in  std_logic;
        aresetn     :   in  std_logic;
        --
        -- Main AXI data to transfer
        --
        axi_addresses   :   in  t_axi_addr_array;
        axi_data        :   in  t_axi_data_array;
        start_i         :   in  std_logic;
        --
        -- Single data to transfer
        --
        axi_addr_single :   in  t_axi_addr;
        axi_data_single :   in  t_axi_data;
        start_single_i  :   in  std_logic_vector(1 downto 0);
        --
        -- Signals
        --
        bus_m           :   out t_axi_bus_master;
        bus_s           :   in  t_axi_bus_slave
    );
end component;

COMPONENT DDS1
  PORT (
    aclk : IN STD_LOGIC;
    aresetn : IN STD_LOGIC;
    s_axis_phase_tvalid : IN STD_LOGIC;
    s_axis_phase_tdata : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
    m_axis_data_tvalid : OUT STD_LOGIC;
    m_axis_data_tdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0) 
  );
END COMPONENT;

--
-- Clocks and reset
--
signal clk_period   :   time    :=  10 ns;
signal sysClk,adcClk,adcClkx2:   std_logic;
signal aresetn      :   std_logic;
--
-- ADC and DAC data
--
signal adcData_i    :   std_logic_vector(31 downto 0);
signal m_axis_tdata :   std_logic_vector(31 downto 0);
signal m_axis_tvalid:   std_logic;
signal pwm_o        :   std_logic_vector(3 downto 0);
--
-- External inputs and outputs
--
signal ext_i,ext_o  :   std_logic_vector(7 downto 0);
--
-- AXI signals
--
signal addr_i                   :   unsigned(AXI_ADDR_WIDTH-1 downto 0);
signal writeData_i, readData_o  :   std_logic_vector(AXI_DATA_WIDTH-1 downto 0);
signal dataValid_i, resp_o      :   std_logic_vector(1 downto 0);
signal bus_m                    :   t_axi_bus_master;
signal bus_s                    :   t_axi_bus_slave;

--
-- AXI data
--

constant axi_addresses   :   t_axi_addr_array(5 downto 0)  :=   (0  =>  X"0000000C",
                                                                 1  =>  X"00000100",
                                                                 2  =>  X"00000104",
                                                                 3  =>  X"00000108",
                                                                 4  =>  X"0000010C",
                                                                 5  =>  X"00000200",
                                                                 6  =>  X"00000204",
                                                                 7  =>  X"00000210",
                                                                 8  =>  X"00000214");
                                                     

signal axi_data :   t_axi_data_array(axi_addresses'length - 1 downto 0);          

signal triggers, outputReg, filterReg          :   t_param_reg;

--
-- AXI control signals
--
signal startAXI     :   std_logic;
signal axi_addr_single  :   t_axi_addr;
signal axi_data_single  :   t_axi_data;
signal start_single_i   :   std_logic_vector(1 downto 0);

signal pidRegs  :   t_param_reg_array(1 downto 0);
signal pidGainRegs  :   t_param_reg_array(1 downto 0);
signal pwmRegs  :   t_param_reg_array(1 downto 0);
signal pwmLimitRegs :   t_param_reg_array(1 downto 0);

signal adc_data :   signed(15 downto 0);

begin

clk_proc: process is
begin
    sysClk <= '0';
    adcClk <= '0';
    wait for clk_period/2;
    sysClk <= '1';
    adcClk <= '1';
    wait for clk_period/2;
end process;

clkx2_proc: process is
begin
    adcClkx2 <= '0';
    wait for clk_period/4;
    adcClkx2 <= '1';
    wait for clk_period/4;
end process;

uut: topmod
port map(
    sysclk          =>  sysclk,
    adcclk          =>  adcclk,
    adcClkx2        =>  adcClkx2,
    aresetn         =>  aresetn,
    addr_i          =>  addr_i,
    writeData_i     =>  writeData_i,
    dataValid_i     =>  dataValid_i,
    readData_o      =>  readData_o,
    resp_o          =>  resp_o,
    ext_i           =>  ext_i,
    ext_o           =>  ext_o,
    pwm_o           =>  pwm_o,
    m_axis_tdata    =>  m_axis_tdata,
    m_axis_tvalid   =>  m_axis_tvalid,
    adcData_i       =>  adcData_i
);

AXI: AXI_Tester
port map(
    clk             =>  sysClk,
    aresetn         =>  aresetn,
    axi_addresses   =>  axi_addresses,
    axi_data        =>  axi_data,
    start_i         =>  startAXI,
    axi_addr_single =>  axi_addr_single,
    axi_data_single =>  axi_data_single,
    start_single_i  =>  start_single_i,
    bus_m           =>  bus_m,
    bus_s           =>  bus_s
);


addr_i <= bus_m.addr;
writeData_i <= bus_m.data;
dataValid_i <= bus_m.valid;
bus_s.data <= readData_o;
bus_s.resp <= resp_o;

--
-- Assign AXI registers
--
axi_data <= (0  =>  filterReg,
             1  =>  pidRegs(0),
             2  =>  pidRegs(1),
             3  =>  pidGainRegs(0),
             4  =>  pidGainRegs(1),
             5  =>  pwmRegs(0),
             6  =>  pwmRegs(1),
             7  =>  pwmLimitRegs(0),
             8  =>  pwmLimitRegs(1)
             );
             
--adc_data <= signed(m_axis_tdata(15 downto 0));
adcData_i <= m_axis_tdata;
--adcData_i <= m_axis_tdata;




main_proc: process is
begin
    --
    -- Initialize the registers and reset
    --
    aresetn <= '0';
    startAXI <= '0';
    ext_i <= (others => '0');
    triggers <= (others => '0');
    outputReg <= (others => '0');
    filterReg <= X"00" & X"00" & X"00" & X"08";
    pidRegs(0) <= std_logic_vector(to_signed(1000,16)) & X"0001";
    pidRegs(1) <= X"0000" & std_logic_vector(to_signed(500,16));
    pidGainRegs(0) <= X"04000101";
    pidGainRegs(1) <= X"04000101";
    pwmRegs(0) <= std_logic_vector(to_signed(0,32));
    pwmRegs(1) <= std_logic_vector(to_signed(0,32));
    pwmLimitRegs(0) <= std_logic_vector(to_unsigned(1000,16)) & std_logic_vector(to_unsigned(0,16));
    pwmLimitRegs(1) <= std_logic_vector(to_unsigned(1000,16)) & std_logic_vector(to_unsigned(0,16));
    
    axi_addr_single <= (others => '0');
    axi_data_single <= (others => '0');
    start_single_i <= "00";
    wait for 200 ns;
    aresetn <= '1';
    wait for 100 ns;
    --
    -- Start AXI transfer
    --
    wait until rising_edge(sysclk);
    startAXI <= '1';
    wait until rising_edge(sysclk);
    startAXI <= '0';
    wait for 10 us;
    --
    -- Change filter rate
    --
    wait until rising_edge(sysclk);
    axi_addr_single <= X"0000_0100";
    axi_data_single <= X"00000003";
    start_single_i <= "01";
    wait until bus_s.resp(0) = '1';
    start_single_i <= "00";
    wait for 500 ns;
    --
    -- Change demodulation phase for 2x freq
    --
--    wait for 50 us;
--    wait until rising_edge(sysclk);
--    axi_addr_single <= X"0000_0028";
--    axi_data_single <= std_logic_vector(shift_left(to_unsigned(1,32),30));
--    start_single_i <= "01";
--    wait until bus_s.resp(0) = '1';
--    start_single_i <= "00";
--    wait for 500 ns;
    

    wait;
end process; 


end Behavioral;
