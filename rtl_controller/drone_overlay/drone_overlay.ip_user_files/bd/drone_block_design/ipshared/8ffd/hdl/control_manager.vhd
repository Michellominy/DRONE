library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity control_manager is
    generic (
        C_S_AXI_DATA_WIDTH : integer := 32;
        -- Increased to 7 bits to support up to 128 bytes (32 registers)
        C_S_AXI_ADDR_WIDTH : integer := 7 
    );
    port (
        -- Global Clock and Reset
        S_AXI_ACLK    : in  std_logic;
        S_AXI_ARESETN : in  std_logic;

        -- AXI4-Lite Write Address Channel
        S_AXI_AWADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_AWVALID : in  std_logic;
        S_AXI_AWREADY : out std_logic;

        -- AXI4-Lite Write Data Channel
        S_AXI_WDATA   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_WSTRB   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
        S_AXI_WVALID  : in  std_logic;
        S_AXI_WREADY  : out std_logic;

        -- AXI4-Lite Write Response Channel
        S_AXI_BRESP   : out std_logic_vector(1 downto 0);
        S_AXI_BVALID  : out std_logic;
        S_AXI_BREADY  : in  std_logic;

        -- AXI4-Lite Read Address Channel
        S_AXI_ARADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_ARVALID : in  std_logic;
        S_AXI_ARREADY : out std_logic;

        -- AXI4-Lite Read Data Channel
        S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_RRESP   : out std_logic_vector(1 downto 0);
        S_AXI_RVALID  : out std_logic;
        S_AXI_RREADY  : in  std_logic;
        
        fusion_ap_start : out std_logic;
        pid_ap_start : out std_logic;
        mixer_ap_start : out std_logic;
        
        -- sensor Controls
        imu_start_init     : out std_logic;
        pca_start_init     : out std_logic;
        imu_ready     : in std_logic;
        pca_ready     : in std_logic;
        
        -- PID Controller Setpoints & Gains (32-bit Float Bits)
        setpoint_roll  : out std_logic_vector(31 downto 0);
        setpoint_pitch : out std_logic_vector(31 downto 0);
        setpoint_yaw   : out std_logic_vector(31 downto 0);
        
        kp_roll        : out std_logic_vector(31 downto 0);
        ki_roll        : out std_logic_vector(31 downto 0);
        kd_roll        : out std_logic_vector(31 downto 0);
        
        kp_pitch       : out std_logic_vector(31 downto 0);
        ki_pitch       : out std_logic_vector(31 downto 0);
        kd_pitch       : out std_logic_vector(31 downto 0);
        
        kp_yaw         : out std_logic_vector(31 downto 0);
        ki_yaw         : out std_logic_vector(31 downto 0);
        kd_yaw         : out std_logic_vector(31 downto 0);
        
        -- Telemetry
        pitch          : in std_logic_vector(31 downto 0);
        roll           : in std_logic_vector(31 downto 0);
        yaw            : in std_logic_vector(31 downto 0);
        motor_1        : in std_logic_vector(31 downto 0);
        motor_2        : in std_logic_vector(31 downto 0);
        motor_3        : in std_logic_vector(31 downto 0);
        motor_4        : in std_logic_vector(31 downto 0);
        
        reset_states   : out std_logic; 
        
        -- Mixer Module Controls
        adj_throttle_m1 : out std_logic_vector(31 downto 0);
        adj_throttle_m2 : out std_logic_vector(31 downto 0);
        adj_throttle_m3 : out std_logic_vector(31 downto 0);
        adj_throttle_m4 : out std_logic_vector(31 downto 0);
        
        -- Emergency Stop
        e_stop_out : out std_logic
    );
end entity control_manager;

architecture rtl of control_manager is

    -- State Machine Definition
    type sys_state_t is (ST_IDLE, ST_INIT, ST_RUNNING, ST_ESTOP);
    signal current_state : sys_state_t := ST_IDLE;

    -- AXI4-Lite Internal Signals
    signal aw_en : std_logic;
    signal axi_awready : std_logic;
    signal axi_wready  : std_logic;
    signal axi_bvalid  : std_logic;
    signal axi_arready : std_logic;
    signal axi_rvalid  : std_logic;
    signal axi_rdata   : std_logic_vector(31 downto 0);

    -- Memory Mapped Registers (Expanded to 0 to 17 to support 4 throttles)
    -- Reg 0: State Machine Command
    -- Reg 1-4: Adjusted Throttles (Motor 1 to 4)
    -- Reg 5-7: Setpoints (Roll, Pitch, Yaw)
    -- Reg 8-16: PID Gains (Roll, Pitch, Yaw)
    -- Reg 17: Reset States Flag
    type reg_array_t is array (0 to 17) of std_logic_vector(31 downto 0);
    signal slv_reg : reg_array_t := (others => (others => '0'));

begin

    -- I/O Connections
    S_AXI_AWREADY <= axi_awready;
    S_AXI_WREADY  <= axi_wready;
    S_AXI_BRESP   <= "00"; -- Always OKAY
    S_AXI_BVALID  <= axi_bvalid;
    S_AXI_ARREADY <= axi_arready;
    S_AXI_RDATA   <= axi_rdata;
    S_AXI_RRESP   <= "00"; -- Always OKAY
    S_AXI_RVALID  <= axi_rvalid;

    ----------------------------------------------------------------------------
    -- AXI4-Lite Write Logic
    ----------------------------------------------------------------------------
    process (S_AXI_ACLK)
        variable loc_addr : integer;
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                axi_awready <= '0';
                axi_wready  <= '0';
                axi_bvalid  <= '0';
                aw_en       <= '1';
                slv_reg     <= (others => (others => '0'));
            else
                -- Accept Address
                if (axi_awready = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1' and aw_en = '1') then
                    axi_awready <= '1';
                    aw_en       <= '0';
                elsif (S_AXI_BREADY = '1' and axi_bvalid = '1') then
                    aw_en       <= '1';
                    axi_awready <= '0';
                else
                    axi_awready <= '0';
                end if;

                -- Accept Data
                if (axi_wready = '0' and S_AXI_WVALID = '1' and S_AXI_AWVALID = '1' and aw_en = '1') then
                    axi_wready <= '1';
                else
                    axi_wready <= '0';
                end if;

                -- Write to Registers
                if (axi_wready = '1' and S_AXI_WVALID = '1' and axi_awready = '1' and S_AXI_AWVALID = '1') then
                    loc_addr := to_integer(unsigned(S_AXI_AWADDR(6 downto 2)));
                    if loc_addr <= 17 then
                        for byte_index in 0 to 3 loop
                            if S_AXI_WSTRB(byte_index) = '1' then
                                slv_reg(loc_addr)(byte_index*8+7 downto byte_index*8) <= S_AXI_WDATA(byte_index*8+7 downto byte_index*8);
                            end if;
                        end loop;
                    end if;
                end if;

                -- Write Response
                if (axi_awready = '1' and S_AXI_AWVALID = '1' and axi_wready = '1' and S_AXI_WVALID = '1' and axi_bvalid = '0') then
                    axi_bvalid <= '1';
                elsif (S_AXI_BREADY = '1' and axi_bvalid = '1') then
                    axi_bvalid <= '0';
                end if;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- AXI4-Lite Read Logic
    ----------------------------------------------------------------------------
    process (S_AXI_ACLK)
        variable loc_addr : integer;
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                axi_arready <= '0';
                axi_rvalid  <= '0';
                axi_rdata   <= (others => '0');
            else
                if (axi_arready = '0' and S_AXI_ARVALID = '1') then
                    axi_arready <= '1';
                    loc_addr := to_integer(unsigned(S_AXI_ARADDR(6 downto 2)));
                    
                    -- Multiplex Read Data Directly
                    if loc_addr <= 17 then
                        axi_rdata <= slv_reg(loc_addr);
                    elsif loc_addr = 18 then
                        axi_rdata(0) <= imu_ready and pca_ready;
                        axi_rdata(31 downto 1) <= (others => '0');
                    elsif loc_addr = 19 then
                        axi_rdata <= pitch;
                    elsif loc_addr = 20 then
                        axi_rdata <= roll;
                    elsif loc_addr = 21 then
                        axi_rdata <= yaw;
                    elsif loc_addr = 22 then
                        axi_rdata <= motor_1;
                    elsif loc_addr = 23 then
                        axi_rdata <= motor_2;
                    elsif loc_addr = 24 then
                        axi_rdata <= motor_3;
                    elsif loc_addr = 25 then
                        axi_rdata <= motor_4;
                    else
                        axi_rdata <= (others => '0');
                    end if;
                else
                    axi_arready <= '0';
                end if;

                if (axi_arready = '1' and S_AXI_ARVALID = '1' and axi_rvalid = '0') then
                    axi_rvalid <= '1';
                elsif (axi_rvalid = '1' and S_AXI_RREADY = '1') then
                    axi_rvalid <= '0';
                end if;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- State Machine & Output Routing
    ----------------------------------------------------------------------------
    process (S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                current_state <= ST_IDLE;
            else
                -- Decode commanded state from Reg 0
                case slv_reg(0) is
                    when x"00000000" => current_state <= ST_IDLE;
                    when x"00000001" => current_state <= ST_INIT;
                    when x"00000002" => current_state <= ST_RUNNING;
                    when x"00000003" => current_state <= ST_ESTOP;
                    when others      => current_state <= ST_ESTOP;
                end case;
            end if;
        end if;
    end process;
    
    e_stop_out <= '1' when current_state = ST_ESTOP else '0';
    
    imu_start_init <= '1' when current_state = ST_INIT else '0';
    pca_start_init <= '1' when current_state = ST_INIT else '0';
    
    fusion_ap_start <= '1' when current_state = ST_RUNNING else '0';
    pid_ap_start <= '1' when current_state = ST_RUNNING else '0';
    mixer_ap_start <= '1' when current_state = ST_RUNNING else '0';

    -- PID state should reset during initialization or if manually requested via Reg 17
    reset_states <= '1' when (current_state = ST_INIT) or (slv_reg(17)(0) = '1') else '0';

    -- Throttle Safety: Only output non-zero throttle when strictly in RUNNING state. 
    adj_throttle_m1 <= slv_reg(1) when current_state = ST_RUNNING else x"00000000";
    adj_throttle_m2 <= slv_reg(2) when current_state = ST_RUNNING else x"00000000";
    adj_throttle_m3 <= slv_reg(3) when current_state = ST_RUNNING else x"00000000";
    adj_throttle_m4 <= slv_reg(4) when current_state = ST_RUNNING else x"00000000";

    -- Pass-through outputs (Updated register mapping indices)
    setpoint_roll  <= slv_reg(5);
    setpoint_pitch <= slv_reg(6);
    setpoint_yaw   <= slv_reg(7);
    
    kp_roll        <= slv_reg(8);
    ki_roll        <= slv_reg(9);
    kd_roll        <= slv_reg(10);
    
    kp_pitch       <= slv_reg(11);
    ki_pitch       <= slv_reg(12);
    kd_pitch       <= slv_reg(13);
    
    kp_yaw         <= slv_reg(14);
    ki_yaw         <= slv_reg(15);
    kd_yaw         <= slv_reg(16);

end architecture rtl;