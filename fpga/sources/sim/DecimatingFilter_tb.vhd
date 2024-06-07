library IEEE;
library work;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;
use work.AXI_Bus_Package.all;

entity DecimatingFilter_tb is
--  Port ( );
end DecimatingFilter_tb;

architecture Behavioral of DecimatingFilter_tb is

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

--
-- Clocks and reset
--
constant clk_period   :   time    :=  10 ns;
signal clk          :   std_logic;
signal aresetn      :   std_logic;
signal filter_reg_i :   t_param_reg;
signal data_i       :   t_adc_array(1 downto 0);
signal data_o       :   t_meas_array(1 downto 0);
signal valid_o      :   std_logic_vector(1 downto 0);


begin

clk_proc: process is
begin
    clk <= '0';
    wait for clk_period/2;
    clk <= '1';
    wait for clk_period/2;
end process;

uut: DecimatingFilter
generic map(
    NUM_INPUT_SIGNALS   =>  2
)
port map(
    clk     =>  clk,
    aresetn =>  aresetn,
    filter_reg_i    =>  filter_reg_i,
    data_i      =>  data_i,
    filtered_data_o =>  data_o,
    valid_o =>  valid_o
);

data_i(0) <= to_signed(1000,t_adc'length);
data_i(1) <= to_signed(-500,t_adc'length);

main: process
begin
    aresetn <= '0';
    filter_Reg_i <= X"0000000a";
    wait for 100 ns;
    aresetn <= '1';
    wait until rising_edge(clk);
    filter_Reg_i <= X"00000008";
    wait;
end process;

end Behavioral;
