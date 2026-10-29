--Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
--Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
----------------------------------------------------------------------------------
--Command: generate_target bd_ea65.bd
--Design : bd_ea65
--Purpose: IP block netlist
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
library UNISIM;
use UNISIM.VCOMPONENTS.ALL;
entity bd_ea65 is
  port (
    SLOT_0_AXIS_tdata : in STD_LOGIC_VECTOR ( 127 downto 0 );
    SLOT_0_AXIS_tkeep : in STD_LOGIC_VECTOR ( 15 downto 0 );
    SLOT_0_AXIS_tlast : in STD_LOGIC;
    SLOT_0_AXIS_tready : in STD_LOGIC;
    SLOT_0_AXIS_tvalid : in STD_LOGIC;
    clk : in STD_LOGIC;
    probe0 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    probe1 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe10 : in STD_LOGIC_VECTOR ( 2 downto 0 );
    probe11 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe12 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe13 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe14 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe15 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe16 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe17 : in STD_LOGIC_VECTOR ( 2 downto 0 );
    probe18 : in STD_LOGIC_VECTOR ( 1 downto 0 );
    probe19 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe2 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe3 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe4 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe5 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    probe6 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe7 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    probe8 : in STD_LOGIC_VECTOR ( 7 downto 0 );
    probe9 : in STD_LOGIC_VECTOR ( 0 to 0 );
    resetn : in STD_LOGIC
  );
  attribute CORE_GENERATION_INFO : string;
  attribute CORE_GENERATION_INFO of bd_ea65 : entity is "bd_ea65,IP_Integrator,{x_ipVendor=xilinx.com,x_ipLibrary=BlockDiagram,x_ipName=bd_ea65,x_ipVersion=1.00.a,x_ipLanguage=VHDL,numBlks=2,numReposBlks=2,numNonXlnxBlks=0,numHierBlks=0,maxHierDepth=0,numSysgenBlks=0,numHlsBlks=0,numHdlrefBlks=0,numPkgbdBlks=0,bdsource=SBD,synth_mode=None}";
  attribute HW_HANDOFF : string;
  attribute HW_HANDOFF of bd_ea65 : entity is "drone_block_design_system_ila_0_0.hwdef";
end bd_ea65;

architecture STRUCTURE of bd_ea65 is
  component bd_ea65_ila_lib_0 is
  port (
    clk : in STD_LOGIC;
    probe0 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    probe1 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe2 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe3 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe4 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe5 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    probe6 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe7 : in STD_LOGIC_VECTOR ( 3 downto 0 );
    probe8 : in STD_LOGIC_VECTOR ( 7 downto 0 );
    probe9 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe10 : in STD_LOGIC_VECTOR ( 2 downto 0 );
    probe11 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe12 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe13 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe14 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe15 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe16 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe17 : in STD_LOGIC_VECTOR ( 2 downto 0 );
    probe18 : in STD_LOGIC_VECTOR ( 1 downto 0 );
    probe19 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe20 : in STD_LOGIC_VECTOR ( 127 downto 0 );
    probe21 : in STD_LOGIC_VECTOR ( 15 downto 0 );
    probe22 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe23 : in STD_LOGIC_VECTOR ( 0 to 0 );
    probe24 : in STD_LOGIC_VECTOR ( 0 to 0 )
  );
  end component bd_ea65_ila_lib_0;
  component bd_ea65_g_inst_0 is
  port (
    aclk : in STD_LOGIC;
    aresetn : in STD_LOGIC;
    slot_0_axis_tvalid : in STD_LOGIC;
    slot_0_axis_tready : in STD_LOGIC;
    slot_0_axis_tdata : in STD_LOGIC_VECTOR ( 127 downto 0 );
    slot_0_axis_tkeep : in STD_LOGIC_VECTOR ( 15 downto 0 );
    slot_0_axis_tlast : in STD_LOGIC;
    m_slot_0_axis_tvalid : out STD_LOGIC;
    m_slot_0_axis_tready : out STD_LOGIC;
    m_slot_0_axis_tdata : out STD_LOGIC_VECTOR ( 127 downto 0 );
    m_slot_0_axis_tkeep : out STD_LOGIC_VECTOR ( 15 downto 0 );
    m_slot_0_axis_tlast : out STD_LOGIC
  );
  end component bd_ea65_g_inst_0;
  signal net_slot_0_axis_tdata : STD_LOGIC_VECTOR ( 127 downto 0 );
  signal net_slot_0_axis_tkeep : STD_LOGIC_VECTOR ( 15 downto 0 );
  signal net_slot_0_axis_tlast : STD_LOGIC;
  signal net_slot_0_axis_tready : STD_LOGIC;
  signal net_slot_0_axis_tvalid : STD_LOGIC;
  attribute X_INTERFACE_INFO : string;
  attribute X_INTERFACE_INFO of SLOT_0_AXIS_tlast : signal is "xilinx.com:interface:axis:1.0 SLOT_0_AXIS TLAST";
  attribute X_INTERFACE_INFO of SLOT_0_AXIS_tready : signal is "xilinx.com:interface:axis:1.0 SLOT_0_AXIS TREADY";
  attribute X_INTERFACE_INFO of SLOT_0_AXIS_tvalid : signal is "xilinx.com:interface:axis:1.0 SLOT_0_AXIS TVALID";
  attribute X_INTERFACE_INFO of clk : signal is "xilinx.com:signal:clock:1.0 CLK.CLK CLK";
  attribute X_INTERFACE_PARAMETER : string;
  attribute X_INTERFACE_PARAMETER of clk : signal is "XIL_INTERFACENAME CLK.CLK, ASSOCIATED_BUSIF SLOT_0_AXIS, ASSOCIATED_RESET resetn, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, INSERT_VIP 0, PHASE 0.0";
  attribute X_INTERFACE_INFO of resetn : signal is "xilinx.com:signal:reset:1.0 RST.RESETN RST";
  attribute X_INTERFACE_PARAMETER of resetn : signal is "XIL_INTERFACENAME RST.RESETN, INSERT_VIP 0, POLARITY ACTIVE_LOW";
  attribute X_INTERFACE_INFO of SLOT_0_AXIS_tdata : signal is "xilinx.com:interface:axis:1.0 SLOT_0_AXIS TDATA";
  attribute X_INTERFACE_MODE : string;
  attribute X_INTERFACE_MODE of SLOT_0_AXIS_tdata : signal is "Monitor SlaveType";
  attribute X_INTERFACE_PARAMETER of SLOT_0_AXIS_tdata : signal is "XIL_INTERFACENAME SLOT_0_AXIS, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, FREQ_HZ 100000000, HAS_TKEEP 1, HAS_TLAST 1, HAS_TREADY 1, HAS_TSTRB 0, INSERT_VIP 0, LAYERED_METADATA undef, PHASE 0.0, TDATA_NUM_BYTES 16, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0";
  attribute X_INTERFACE_INFO of SLOT_0_AXIS_tkeep : signal is "xilinx.com:interface:axis:1.0 SLOT_0_AXIS TKEEP";
begin
g_inst: component bd_ea65_g_inst_0
     port map (
      aclk => clk,
      aresetn => resetn,
      m_slot_0_axis_tdata(127 downto 0) => net_slot_0_axis_tdata(127 downto 0),
      m_slot_0_axis_tkeep(15 downto 0) => net_slot_0_axis_tkeep(15 downto 0),
      m_slot_0_axis_tlast => net_slot_0_axis_tlast,
      m_slot_0_axis_tready => net_slot_0_axis_tready,
      m_slot_0_axis_tvalid => net_slot_0_axis_tvalid,
      slot_0_axis_tdata(127 downto 0) => SLOT_0_AXIS_tdata(127 downto 0),
      slot_0_axis_tkeep(15 downto 0) => SLOT_0_AXIS_tkeep(15 downto 0),
      slot_0_axis_tlast => SLOT_0_AXIS_tlast,
      slot_0_axis_tready => SLOT_0_AXIS_tready,
      slot_0_axis_tvalid => SLOT_0_AXIS_tvalid
    );
ila_lib: component bd_ea65_ila_lib_0
     port map (
      clk => clk,
      probe0(3 downto 0) => probe0(3 downto 0),
      probe1(0) => probe1(0),
      probe10(2 downto 0) => probe10(2 downto 0),
      probe11(0) => probe11(0),
      probe12(0) => probe12(0),
      probe13(0) => probe13(0),
      probe14(0) => probe14(0),
      probe15(0) => probe15(0),
      probe16(0) => probe16(0),
      probe17(2 downto 0) => probe17(2 downto 0),
      probe18(1 downto 0) => probe18(1 downto 0),
      probe19(0) => probe19(0),
      probe2(0) => probe2(0),
      probe20(127 downto 0) => net_slot_0_axis_tdata(127 downto 0),
      probe21(15 downto 0) => net_slot_0_axis_tkeep(15 downto 0),
      probe22(0) => net_slot_0_axis_tvalid,
      probe23(0) => net_slot_0_axis_tready,
      probe24(0) => net_slot_0_axis_tlast,
      probe3(0) => probe3(0),
      probe4(0) => probe4(0),
      probe5(3 downto 0) => probe5(3 downto 0),
      probe6(0) => probe6(0),
      probe7(3 downto 0) => probe7(3 downto 0),
      probe8(7 downto 0) => probe8(7 downto 0),
      probe9(0) => probe9(0)
    );
end STRUCTURE;
