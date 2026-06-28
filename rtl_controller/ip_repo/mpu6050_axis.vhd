library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity mpu6050_axis is
    generic(
        CLK_FREQ : integer := 100_000_000;
        I2C_FREQ : integer := 400_000
    );
    port(
        clk           : in    std_logic;
        reset_n       : in    std_logic;

        mpu_int       : in    std_logic;

        -- Separated I2C Tristate control signals
        scl_i         : in    std_logic;
        scl_t         : out   std_logic;
        sda_i         : in    std_logic;
        sda_t         : out   std_logic;

        m_axis_tdata  : out   std_logic_vector(127 downto 0);
        m_axis_tvalid : out   std_logic;
        m_axis_tready : in    std_logic;
        m_axis_tlast  : out   std_logic;
        m_axis_tkeep  : out   std_logic_vector(15 downto 0);
        
        debug_mpu_state    : out std_logic_vector(3 downto 0); 
        debug_raw_byte     : out std_logic_vector(7 downto 0)

    );
end entity;

architecture rtl of mpu6050_axis is

    type reg_config_t is record
        addr : std_logic_vector(7 downto 0);
        data : std_logic_vector(7 downto 0);
    end record;
    
    type init_regs_t is array (0 to 5) of reg_config_t;
    constant INIT_REGS : init_regs_t := (
        (x"6B", x"01"), -- PWR_MGMT_1
        (x"19", x"07"), -- SMPLRT_DIV
        (x"1A", x"04"), -- CONFIG
        (x"1B", x"18"), -- GYRO_CONFIG
        (x"37", x"30"), -- INT_PIN_CFG
        (x"38", x"01")  -- INT_ENABLE
    );

    type mpu_state_t is (
        ST_RESET, ST_HW_RESET_CMD, ST_HW_RESET_WAIT, ST_HW_RESET_DELAY,
        ST_INIT_CMD, ST_INIT_WAIT, ST_WAIT_INT, 
        ST_BURST_CMD, ST_BURST_WAIT, ST_STREAM
    );
    signal mpu_state : mpu_state_t := ST_RESET;

    signal i2c_rst        : std_logic;
    signal i2c_cmd_valid_r: std_logic := '0';
    signal i2c_cmd_ready  : std_logic;

    signal i2c_cmd_type   : std_logic_vector(1 downto 0) := "00";
    signal i2c_reg_addr   : std_logic_vector(7 downto 0) := (others => '0');
    signal i2c_wr_data    : std_logic_vector(7 downto 0) := (others => '0');
    signal i2c_rd_data    : std_logic_vector(7 downto 0);
    signal i2c_burst_len  : unsigned(7 downto 0) := (others => '0');
    signal i2c_byte_valid : std_logic;
    signal i2c_done       : std_logic;

    signal i2c_busy       : std_logic;
    signal i2c_error      : std_logic;

    type raw_bytes_t is array (0 to 13) of std_logic_vector(7 downto 0);
    signal raw_bytes : raw_bytes_t := (others => (others => '0'));
    
    signal init_idx  : integer range 0 to 5 := 0;
    signal byte_idx  : integer range 0 to 13 := 0;

    constant TIMEOUT_CYCLES : integer := CLK_FREQ / 10;
    signal timeout_cnt      : integer range 0 to TIMEOUT_CYCLES := 0;
    signal delay_cnt        : integer range 0 to TIMEOUT_CYCLES := 0;
    
    -- 2-Stage Synchronizer for the asynchronous interrupt pin
    signal mpu_int_m        : std_logic := '0';
    signal mpu_int_sync     : std_logic := '0';

    -- AXI Stream Drive signals
    signal m_valid_reg : std_logic := '0';
    signal m_last_reg  : std_logic := '0';

    signal ax_raw, ay_raw, az_raw : std_logic_vector(15 downto 0);
    signal temp_raw               : std_logic_vector(15 downto 0);
    signal gx_raw, gy_raw, gz_raw : std_logic_vector(15 downto 0);

begin

    i2c_rst <= not reset_n;
    m_axis_tvalid <= m_valid_reg;
    m_axis_tlast  <= m_last_reg;

    -- Pack I2C big-endian into contiguous 16-bit blocks
    ax_raw <= raw_bytes(0)  & raw_bytes(1);
    ay_raw <= raw_bytes(2)  & raw_bytes(3);
    az_raw <= raw_bytes(4)  & raw_bytes(5);
    temp_raw <= raw_bytes(6)  & raw_bytes(7);
    gx_raw <= raw_bytes(8)  & raw_bytes(9);
    gy_raw <= raw_bytes(10) & raw_bytes(11);
    gz_raw <= raw_bytes(12) & raw_bytes(13);

    --------------------------------------------------------------------------
    -- I2C Instance
    --------------------------------------------------------------------------
    u_i2c_master : entity work.i2c_master
    port map(
        clk => clk,
        rst => i2c_rst,
        cmd_valid => i2c_cmd_valid_r,
        cmd_ready => i2c_cmd_ready,
        cmd_type  => i2c_cmd_type,
        dev_addr  => "1101000",
        reg_addr  => i2c_reg_addr,
        wr_data   => i2c_wr_data,
        burst_len => i2c_burst_len,
        rd_data   => i2c_rd_data,
        byte_valid => i2c_byte_valid,
        done       => i2c_done,
        busy       => i2c_busy,
        error      => i2c_error,
        
        -- Tristate Interface
        scl_i => scl_i,
        scl_t => scl_t,
        sda_i => sda_i,
        sda_t => sda_t

    );

    process(clk)
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                mpu_state       <= ST_RESET;
                i2c_cmd_valid_r <= '0';
                init_idx        <= 0;
                byte_idx        <= 0;
                timeout_cnt     <= 0;
                delay_cnt       <= 0;
                m_valid_reg     <= '0';
                m_last_reg      <= '0';
                m_axis_tkeep    <= (others => '0');
                m_axis_tdata    <= (others => '0');
                raw_bytes       <= (others => (others => '0'));
                mpu_int_m       <= '0';
                mpu_int_sync    <= '0';
            else
                mpu_int_m    <= mpu_int;
                mpu_int_sync <= mpu_int_m;

                case mpu_state is
                    when ST_RESET =>
                        init_idx      <= 0;
                        byte_idx      <= 0;
                        timeout_cnt   <= 0;
                        delay_cnt     <= 0;
                        m_valid_reg   <= '0';
                        m_last_reg    <= '0';
                        mpu_state     <= ST_HW_RESET_CMD;

                    when ST_HW_RESET_CMD =>
                        i2c_cmd_type    <= "00";
                        i2c_reg_addr    <= x"6B";
                        i2c_wr_data     <= x"80";
                        i2c_cmd_valid_r <= '1';
                        timeout_cnt     <= 0; 
                        
                        if i2c_cmd_valid_r = '1' and i2c_cmd_ready = '1' then
                            i2c_cmd_valid_r <= '0';
                            mpu_state       <= ST_HW_RESET_WAIT;
                        end if;

                    when ST_HW_RESET_WAIT =>
                        if timeout_cnt = TIMEOUT_CYCLES then
                            mpu_state <= ST_RESET;
                        elsif i2c_done = '1' then
                            timeout_cnt <= 0;
                            mpu_state   <= ST_HW_RESET_DELAY;
                        else
                            timeout_cnt <= timeout_cnt + 1;
                        end if;

                    when ST_HW_RESET_DELAY =>
                        if delay_cnt = TIMEOUT_CYCLES then
                            delay_cnt <= 0;
                            mpu_state <= ST_INIT_CMD;
                        else
                            delay_cnt <= delay_cnt + 1;
                        end if;

                    when ST_INIT_CMD =>
                        i2c_cmd_type    <= "00";
                        i2c_reg_addr    <= INIT_REGS(init_idx).addr;
                        i2c_wr_data     <= INIT_REGS(init_idx).data;
                        i2c_cmd_valid_r <= '1';
                        timeout_cnt     <= 0; 
                        
                        if i2c_cmd_valid_r = '1' and i2c_cmd_ready = '1' then
                            i2c_cmd_valid_r <= '0';
                            mpu_state       <= ST_INIT_WAIT;
                        end if;

                    when ST_INIT_WAIT =>
                        if timeout_cnt = TIMEOUT_CYCLES then
                            mpu_state <= ST_RESET;
                        elsif i2c_done = '1' then
                            timeout_cnt <= 0;
                            if init_idx = 5 then
                                mpu_state <= ST_WAIT_INT;
                            else
                                init_idx  <= init_idx + 1;
                                mpu_state <= ST_INIT_CMD;
                            end if;
                        else
                            timeout_cnt <= timeout_cnt + 1;
                        end if;

                    when ST_WAIT_INT =>
                        byte_idx <= 0;
                        if mpu_int_sync = '1' then
                            mpu_state <= ST_BURST_CMD;
                        end if;

                    when ST_BURST_CMD =>
                        i2c_cmd_type    <= "10";
                        i2c_reg_addr    <= x"3B";
                        i2c_burst_len   <= to_unsigned(14, 8);
                        i2c_cmd_valid_r <= '1';
                        timeout_cnt     <= 0; 

                        if i2c_cmd_valid_r = '1' and i2c_cmd_ready = '1' then
                            i2c_cmd_valid_r <= '0';
                            mpu_state       <= ST_BURST_WAIT;
                        end if;

                    when ST_BURST_WAIT =>
                        if timeout_cnt = TIMEOUT_CYCLES then
                            mpu_state <= ST_RESET;
                        else
                            timeout_cnt <= timeout_cnt + 1;
                            
                            if i2c_byte_valid = '1' then
                                raw_bytes(byte_idx) <= i2c_rd_data;
                                if byte_idx < 13 then
                                    byte_idx <= byte_idx + 1;
                                end if;
                            end if;
                            
                            if i2c_done = '1' then
                                timeout_cnt  <= 0;
                                m_valid_reg  <= '1';
                                m_last_reg   <= '1'; 
                                m_axis_tkeep <= x"FFFF";
                                
                                m_axis_tdata(15 downto 0)    <= ax_raw;
                                m_axis_tdata(31 downto 16)   <= std_logic_vector(-signed(ay_raw)); 
                                m_axis_tdata(47 downto 32)   <= std_logic_vector(-signed(az_raw));
                                m_axis_tdata(63 downto 48)   <= temp_raw;
                                m_axis_tdata(79 downto 64)   <= gx_raw;
                                m_axis_tdata(95 downto 80)   <= std_logic_vector(-signed(gy_raw)); 
                                m_axis_tdata(111 downto 96)  <= std_logic_vector(-signed(gz_raw)); 
                                m_axis_tdata(127 downto 112) <= (others => '0');
                                
                                mpu_state <= ST_STREAM;
                            end if;
                        end if;

                    when ST_STREAM =>
                        if m_axis_tready = '1' then
                            m_valid_reg <= '0';
                            m_last_reg  <= '0';
                            mpu_state   <= ST_WAIT_INT;
                        end if;
                end case;
            end if;
        end if;
    end process;

    with mpu_state select
    debug_mpu_state <=
        "0000" when ST_RESET, "0001" when ST_HW_RESET_CMD, "0010" when ST_HW_RESET_WAIT,
        "0011" when ST_HW_RESET_DELAY, "0100" when ST_INIT_CMD, "0101" when ST_INIT_WAIT,
        "0110" when ST_WAIT_INT, "0111" when ST_BURST_CMD, "1000" when ST_BURST_WAIT,
        "1001" when ST_STREAM;

    debug_raw_byte   <= i2c_rd_data;
    
end architecture;