-- (c) Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
-- (c) Copyright 2022-2026 Advanced Micro Devices, Inc. All rights reserved.
-- 
-- This file contains confidential and proprietary information
-- of AMD and is protected under U.S. and international copyright
-- and other intellectual property laws.
-- 
-- DISCLAIMER
-- This disclaimer is not a license and does not grant any
-- rights to the materials distributed herewith. Except as
-- otherwise provided in a valid license issued to you by
-- AMD, and to the maximum extent permitted by applicable
-- law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
-- WITH ALL FAULTS, AND AMD HEREBY DISCLAIMS ALL WARRANTIES
-- AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
-- BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
-- INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
-- (2) AMD shall not be liable (whether in contract or tort,
-- including negligence, or under any other theory of
-- liability) for any loss or damage of any kind or nature
-- related to, arising under or in connection with these
-- materials, including for any direct, or any indirect,
-- special, incidental, or consequential loss or damage
-- (including loss of data, profits, goodwill, or any type of
-- loss or damage suffered as a result of any action brought
-- by a third party) even if such damage or loss was
-- reasonably foreseeable or AMD had been advised of the
-- possibility of the same.
-- 
-- CRITICAL APPLICATIONS
-- AMD products are not designed or intended to be fail-
-- safe, or for use in any application requiring fail-safe
-- performance, such as life-support or safety devices or
-- systems, Class III medical devices, nuclear facilities,
-- applications related to the deployment of airbags, or any
-- other applications that could lead to death, personal
-- injury, or severe property or environmental damage
-- (individually and collectively, "Critical
-- Applications"). Customer assumes the sole risk and
-- liability of any use of AMD products in Critical
-- Applications, subject only to applicable laws and
-- regulations governing limitations on product liability.
-- 
-- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
-- PART OF THIS FILE AT ALL TIMES.
-- 
-- DO NOT MODIFY THIS FILE.

-- IP VLNV: xilinx.com:user:mpu6050_axis_ip:1.0
-- IP Revision: 39

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY drone_block_design_mpu6050_axis_ip_0_0 IS
  PORT (
    clk : IN STD_LOGIC;
    reset_n : IN STD_LOGIC;
    mpu_int : IN STD_LOGIC;
    scl_i : IN STD_LOGIC;
    scl_t : OUT STD_LOGIC;
    sda_i : IN STD_LOGIC;
    sda_t : OUT STD_LOGIC;
    m_axis_tdata : OUT STD_LOGIC_VECTOR(127 DOWNTO 0);
    m_axis_tvalid : OUT STD_LOGIC;
    m_axis_tready : IN STD_LOGIC;
    m_axis_tlast : OUT STD_LOGIC;
    m_axis_tkeep : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
    debug_mpu_state : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
    debug_raw_byte : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
  );
END drone_block_design_mpu6050_axis_ip_0_0;

ARCHITECTURE drone_block_design_mpu6050_axis_ip_0_0_arch OF drone_block_design_mpu6050_axis_ip_0_0 IS
  ATTRIBUTE DowngradeIPIdentifiedWarnings : STRING;
  ATTRIBUTE DowngradeIPIdentifiedWarnings OF drone_block_design_mpu6050_axis_ip_0_0_arch: ARCHITECTURE IS "yes";
  COMPONENT mpu6050_axis IS
    GENERIC (
      CLK_FREQ : INTEGER;
      I2C_FREQ : INTEGER
    );
    PORT (
      clk : IN STD_LOGIC;
      reset_n : IN STD_LOGIC;
      mpu_int : IN STD_LOGIC;
      scl_i : IN STD_LOGIC;
      scl_t : OUT STD_LOGIC;
      sda_i : IN STD_LOGIC;
      sda_t : OUT STD_LOGIC;
      m_axis_tdata : OUT STD_LOGIC_VECTOR(127 DOWNTO 0);
      m_axis_tvalid : OUT STD_LOGIC;
      m_axis_tready : IN STD_LOGIC;
      m_axis_tlast : OUT STD_LOGIC;
      m_axis_tkeep : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
      debug_mpu_state : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);
      debug_raw_byte : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
    );
  END COMPONENT mpu6050_axis;
  ATTRIBUTE X_CORE_INFO : STRING;
  ATTRIBUTE X_CORE_INFO OF drone_block_design_mpu6050_axis_ip_0_0_arch: ARCHITECTURE IS "mpu6050_axis,Vivado 2025.2";
  ATTRIBUTE CHECK_LICENSE_TYPE : STRING;
  ATTRIBUTE CHECK_LICENSE_TYPE OF drone_block_design_mpu6050_axis_ip_0_0_arch : ARCHITECTURE IS "drone_block_design_mpu6050_axis_ip_0_0,mpu6050_axis,{}";
  ATTRIBUTE CORE_GENERATION_INFO : STRING;
  ATTRIBUTE CORE_GENERATION_INFO OF drone_block_design_mpu6050_axis_ip_0_0_arch: ARCHITECTURE IS "drone_block_design_mpu6050_axis_ip_0_0,mpu6050_axis,{x_ipProduct=Vivado 2025.2,x_ipVendor=xilinx.com,x_ipLibrary=user,x_ipName=mpu6050_axis_ip,x_ipVersion=1.0,x_ipCoreRevision=39,x_ipLanguage=VHDL,x_ipSimLanguage=MIXED,CLK_FREQ=100000000,I2C_FREQ=400000}";
  ATTRIBUTE X_INTERFACE_INFO : STRING;
  ATTRIBUTE X_INTERFACE_MODE : STRING;
  ATTRIBUTE X_INTERFACE_PARAMETER : STRING;
  ATTRIBUTE X_INTERFACE_INFO OF clk: SIGNAL IS "xilinx.com:signal:clock:1.0 clk CLK";
  ATTRIBUTE X_INTERFACE_MODE OF clk: SIGNAL IS "slave clk";
  ATTRIBUTE X_INTERFACE_PARAMETER OF clk: SIGNAL IS "XIL_INTERFACENAME clk, ASSOCIATED_BUSIF m_axis, FREQ_HZ 100000000, FREQ_TOLERANCE_HZ 0, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, INSERT_VIP 0";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 m_axis TDATA";
  ATTRIBUTE X_INTERFACE_MODE OF m_axis_tdata: SIGNAL IS "master m_axis";
  ATTRIBUTE X_INTERFACE_PARAMETER OF m_axis_tdata: SIGNAL IS "XIL_INTERFACENAME m_axis, TDATA_NUM_BYTES 16, TDEST_WIDTH 0, TID_WIDTH 0, TUSER_WIDTH 0, HAS_TREADY 1, HAS_TSTRB 0, HAS_TKEEP 1, HAS_TLAST 1, FREQ_HZ 100000000, PHASE 0.0, CLK_DOMAIN drone_block_design_processing_system7_0_0_FCLK_CLK0, LAYERED_METADATA undef, INSERT_VIP 0";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_tkeep: SIGNAL IS "xilinx.com:interface:axis:1.0 m_axis TKEEP";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_tlast: SIGNAL IS "xilinx.com:interface:axis:1.0 m_axis TLAST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 m_axis TREADY";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 m_axis TVALID";
  ATTRIBUTE X_INTERFACE_INFO OF reset_n: SIGNAL IS "xilinx.com:signal:reset:1.0 reset_n RST";
  ATTRIBUTE X_INTERFACE_MODE OF reset_n: SIGNAL IS "slave reset_n";
  ATTRIBUTE X_INTERFACE_PARAMETER OF reset_n: SIGNAL IS "XIL_INTERFACENAME reset_n, POLARITY ACTIVE_LOW, INSERT_VIP 0";
BEGIN
  U0 : mpu6050_axis
    GENERIC MAP (
      CLK_FREQ => 100000000,
      I2C_FREQ => 400000
    )
    PORT MAP (
      clk => clk,
      reset_n => reset_n,
      mpu_int => mpu_int,
      scl_i => scl_i,
      scl_t => scl_t,
      sda_i => sda_i,
      sda_t => sda_t,
      m_axis_tdata => m_axis_tdata,
      m_axis_tvalid => m_axis_tvalid,
      m_axis_tready => m_axis_tready,
      m_axis_tlast => m_axis_tlast,
      m_axis_tkeep => m_axis_tkeep,
      debug_mpu_state => debug_mpu_state,
      debug_raw_byte => debug_raw_byte
    );
END drone_block_design_mpu6050_axis_ip_0_0_arch;
