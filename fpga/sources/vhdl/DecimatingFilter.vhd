library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;
use work.AXI_Bus_Package.all;

entity DecimatingFilter is
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
end DecimatingFilter;

architecture Behavioral of DecimatingFilter is

COMPONENT CICFilter
  PORT (
    aclk : IN STD_LOGIC;
    aresetn : IN STD_LOGIC;
    s_axis_config_tdata : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    s_axis_config_tvalid : IN STD_LOGIC;
    s_axis_config_tready : OUT STD_LOGIC;
    s_axis_data_tdata : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    s_axis_data_tvalid : IN STD_LOGIC;
    s_axis_data_tready : OUT STD_LOGIC;
    m_axis_data_tdata : OUT STD_LOGIC_VECTOR(55 DOWNTO 0);
    m_axis_data_tvalid : OUT STD_LOGIC 
  );
END COMPONENT;

--
-- CIC filter outputs
--
constant CIC_OUTPUT_WIDTH   :   natural :=  56;
constant CIC_ACTUAL_WIDTH   :   natural :=  53;
type t_cic_o_array is array(natural range <>) of std_logic_vector(CIC_OUTPUT_WIDTH - 1 downto 0);
type t_cic_array is array(natural range <>) of std_logic_vector(CIC_ACTUAL_WIDTH - 1 downto 0);
type t_filter_data_i is array(natural range <>) of std_logic_vector(15 downto 0);

--
-- Filter signals
--
signal filter_data_i                    :   t_filter_data_i(NUM_INPUT_SIGNALS - 1 downto 0);
signal cicLog2Rate                      :   unsigned(3 downto 0);
signal cicShift                         :   integer;
signal setShift                         :   signed(7 downto 0);
signal filterConfig, filterConfig_old   :   std_logic_vector(15 downto 0);
signal valid_config                     :   std_logic;
signal filter_o                         :   t_cic_o_array(NUM_INPUT_SIGNALS - 1 downto 0);
signal valid_filter_o                   :   std_logic_vector(NUM_INPUT_SIGNALS - 1 downto 0);
signal filtered_data                    :   t_cic_array(NUM_INPUT_SIGNALS - 1 downto 0);

begin
--
-- Parse registers
--
cicLog2Rate <= unsigned(filter_reg_i(3 downto 0));
setShift <= signed(filter_reg_i(11 downto 4));
--
-- Implement filters
--
cicShift <= to_integer(cicLog2Rate)+ to_integer(cicLog2Rate)+ to_integer(cicLog2Rate);
filterConfig <= std_logic_vector(shift_left(to_unsigned(1, filterConfig'length),to_integer(cicLog2Rate)));
--
-- This creates a signal that is high for a single clock cycle when the
-- filter rate changes
--
ChangeProc: process(clk, aresetn) is
begin 
   if aresetn ='0' then
      filterConfig_old <= filterConfig;
      valid_config <= '0';
   elsif rising_edge(clk) then 
      filterConfig_old <= filterConfig;
      if filterConfig /= filterConfig_old then
        valid_config <= '1';
      else
        valid_config <= '0';
      end if;
   end if;      
end process;
--
-- Procedurally generate all CIC filters
--
FILT_GEN: for I in 0 to NUM_INPUT_SIGNALS - 1 generate
    filter_data_i(I)(ADC_WIDTH - 1 downto 0) <= std_logic_vector(data_i(I));
    filter_data_i(I)(15 downto ADC_WIDTH) <= (others => '0');
    Filt_X : CICfilter
    PORT MAP (
        aclk                        => clk,
        aresetn                     => aresetn,
        s_axis_config_tdata         => filterConfig,
        s_axis_config_tvalid        => valid_config,
        s_axis_config_tready        => open,
        s_axis_data_tdata           => filter_data_i(I),
        s_axis_data_tvalid          => '1',
        s_axis_data_tready          => open,
        m_axis_data_tdata           => filter_o(I),
        m_axis_data_tvalid          => valid_filter_o(I)
    );
--    filtered_data(I) <= filter_o(I)(CIC_ACTUAL_WIDTH - 1 downto 0);
    filtered_data_o(I) <= resize(shift_right(signed(filter_o(I)(CIC_ACTUAL_WIDTH - 1 downto 0)),cicShift + to_integer(setShift)),t_meas'length);
end generate FILT_GEN;

valid_o <= valid_filter_o;

end Behavioral;
