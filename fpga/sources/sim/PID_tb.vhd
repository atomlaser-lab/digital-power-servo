library IEEE;
library work;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;
use work.AXI_Bus_Package.all;

entity PID_tb is

end PID_tb;

architecture rtl of PID_tb is

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

constant clk_period   :   time    :=  10 ns;
signal clk          :   std_logic;
signal aresetn      :   std_logic;

signal meas_i, control_i    :   t_meas;
signal valid_i  :   std_logic;
signal enable_i, polarity_i :   std_logic;
signal gains    :   t_param_reg;
signal valid_o  :   std_logic;
signal data_o   :   t_pwm_exp;


begin

clk_proc: process is
begin
    clk <= '0';
    wait for clk_period/2;
    clk <= '1';
    wait for clk_period/2;
end process;

Valid_proc: process is
begin
    valid_i <= '0';
    wait for 1 us;
    wait until rising_edge(clk);
    valid_i <= '1';
    wait until rising_edge(clk);
    valid_i <= '0';
end process;

uut: PIDController
port map(
    clk =>  clk,
    aresetn =>  aresetn,
    meas_i  =>  meas_i,
    control_i   =>  control_i,
    valid_i =>  valid_i,
    enable_i => enable_i,
    polarity_i => polarity_i,
    hold_i => '0',
    gains => gains,
    valid_o =>  valid_o,
    data_o  =>  data_o
);

meas_i <= resize(data_o,meas_i'length);
control_i <= to_signed(500,control_i'length);

main: process
begin
    aresetn <= '0';
    enable_i <= '0';
    polarity_i <= '0';
    gains <= X"04000a01";
    wait for 100 ns;
    aresetn <= '1';
    wait until rising_edge(clk);
    wait for 100 ns;
    enable_i <= '1';
    wait;
end process;

end rtl;