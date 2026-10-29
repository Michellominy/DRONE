library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity imu_axis is
    generic(
        CLK_FREQ : integer := 100_000_000;
        I2C_FREQ : integer := 400_000
    );
    port(
        clk           : in    std_logic;
        reset_n       : in    std_logic;

        imu_int       : in    std_logic;
        
        -- Separated I2C Tristate control signals
        scl_i         : in    std_logic;
        scl_t         : out   std_logic;
        sda_i         : in    std_logic;
        sda_t         : out   std_logic;

        -- Accel(3), Gyro(3), Mag(3), and Temp(1)
        m_axis_tdata  : out   std_logic_vector(255 downto 0);
        m_axis_tvalid : out   std_logic;
        m_axis_tready : in    std_logic;
        m_axis_tlast  : out   std_logic;
        m_axis_tkeep  : out   std_logic_vector(31 downto 0);
        m_axis_tstrb  : out   std_logic_vector(31 downto 0);
        
        start_init    : in  std_logic;
        ready         : out std_logic
    );
end entity;

architecture rtl of imu_axis is

    type reg_config_t is record
        addr : std_logic_vector(7 downto 0);
        data : std_logic_vector(7 downto 0);
    end record;
    
    -- Array size increased to 17 to accommodate an extra wake-up delay
    type init_regs_t is array (0 to 16) of reg_config_t;
    constant INIT_REGS : init_regs_t := (
        (x"7F", x"00"), -- 0: REG_BANK_SEL -> Bank 0
        (x"06", x"01"), -- 1: PWR_MGMT_1 -> Auto clock (Wake up)
        (x"FF", x"00"), -- 2: DUMMY DELAY -> 100ms for IMU oscillators to stabilize
        (x"03", x"20"), -- 3: USER_CTRL -> I2C_MST_EN
        (x"7F", x"30"), -- 4: REG_BANK_SEL -> Bank 3
        (x"01", x"07"), -- 5: I2C_MST_CTRL -> 345kHz I2C
        (x"13", x"0C"), -- 6: I2C_SLV4_ADDR -> Mag Write (0x0C)
        (x"14", x"31"), -- 7: I2C_SLV4_REG -> MAG_CNTL2
        (x"16", x"08"), -- 8: I2C_SLV4_DO -> 100Hz Continuous Mode 4
        (x"15", x"80"), -- 9: I2C_SLV4_CTRL -> Enable SLV4 transaction
        (x"FF", x"00"), -- 10: DUMMY DELAY -> 100ms for Mag I2C transaction to finish
        (x"03", x"8C"), -- 11: I2C_SLV0_ADDR -> Mag Read (0x0C | 0x80)
        (x"04", x"11"), -- 12: I2C_SLV0_REG -> MAG_HXL
        (x"05", x"88"), -- 13: I2C_SLV0_CTRL -> Enable SLV0, read 8 bytes continuously
        (x"7F", x"00"), -- 14: REG_BANK_SEL -> Bank 0
        (x"0F", x"10"), -- 15: INT_PIN_CFG -> Active high, push-pull, any read clears
        (x"11", x"01")  -- 16: INT_ENABLE_1 -> RAW_DATA_0_RDY_EN
    );

    type imu_state_t is (
        ST_IDLE, ST_RESET, ST_HW_RESET_CMD, ST_HW_RESET_WAIT, 
        ST_HW_RESET_DELAY, ST_INIT_CMD, ST_INIT_WAIT, ST_INIT_DELAY, 
        ST_CALIB_WAIT_INT, ST_CALIB_BURST_CMD, ST_CALIB_BURST_WAIT,
        ST_WAIT_INT, ST_BURST_CMD, ST_BURST_WAIT, ST_STREAM
    );
    signal imu_state : imu_state_t := ST_IDLE;

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

    type raw_bytes_t is array (0 to 21) of std_logic_vector(7 downto 0);
    signal raw_bytes : raw_bytes_t := (others => (others => '0'));
    
    signal init_idx  : integer range 0 to 16 := 0;
    signal byte_idx  : integer range 0 to 21 := 0;

    constant TIMEOUT_CYCLES : integer := CLK_FREQ / 10;
    signal timeout_cnt      : integer range 0 to TIMEOUT_CYCLES := 0;
    signal delay_cnt        : integer range 0 to TIMEOUT_CYCLES := 0;
    
    -- 2-Stage Synchronizer for the asynchronous interrupt pin
    signal imu_int_m        : std_logic := '0';
    signal imu_int_sync     : std_logic := '0';

    -- AXI Stream Drive signals
    signal m_valid_reg : std_logic := '0';
    signal m_last_reg  : std_logic := '0';

    signal ax_raw, ay_raw, az_raw : std_logic_vector(15 downto 0);
    signal temp_raw               : std_logic_vector(15 downto 0);
    signal gx_raw, gy_raw, gz_raw : std_logic_vector(15 downto 0);
    signal mx_raw, my_raw, mz_raw : std_logic_vector(15 downto 0);
    
    constant GYRO_CALIB_SAMPLES : integer := 1024; -- 1024 (Power of 2)
    signal gyro_x_sum : signed(31 downto 0);
    signal gyro_y_sum : signed(31 downto 0);
    signal gyro_z_sum : signed(31 downto 0);
    
    signal gyro_x_offset_reg : signed(15 downto 0);
    signal gyro_y_offset_reg : signed(15 downto 0);
    signal gyro_z_offset_reg : signed(15 downto 0);
    
    signal gyro_calib_count : integer range 0 to GYRO_CALIB_SAMPLES;
    
begin

    i2c_rst <= not reset_n;
    m_axis_tvalid <= m_valid_reg;
    m_axis_tlast  <= m_last_reg;

    -- IMU unpack: Big-Endian format
    ax_raw   <= raw_bytes(0)  & raw_bytes(1);
    ay_raw   <= raw_bytes(2)  & raw_bytes(3);
    az_raw   <= raw_bytes(4)  & raw_bytes(5);
    gx_raw   <= raw_bytes(6)  & raw_bytes(7);
    gy_raw   <= raw_bytes(8)  & raw_bytes(9);
    gz_raw   <= raw_bytes(10) & raw_bytes(11);
    temp_raw <= raw_bytes(12) & raw_bytes(13);

    -- Magnetometer unpack: Little-Endian format (0x11 is ST1, actual data starts at offset 15)
    mx_raw   <= raw_bytes(16) & raw_bytes(15);
    my_raw   <= raw_bytes(18) & raw_bytes(17);
    mz_raw   <= raw_bytes(20) & raw_bytes(19);
    
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
        dev_addr  => "1101001", -- ICM-20948 Address (0x69)
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
                imu_state       <= ST_IDLE;
                i2c_cmd_valid_r <= '0';
                init_idx        <= 0;
                byte_idx        <= 0;
                timeout_cnt     <= 0;
                delay_cnt       <= 0;
                m_valid_reg     <= '0';
                m_last_reg      <= '0';
                m_axis_tkeep    <= (others => '0');
                m_axis_tstrb    <= (others => '0');
                m_axis_tdata    <= (others => '0');
                raw_bytes       <= (others => (others => '0'));
                imu_int_m       <= '0';
                imu_int_sync    <= '0';
                
                ready <= '0';
                
                gyro_x_sum <= (others => '0');
                gyro_y_sum <= (others => '0');
                gyro_z_sum <= (others => '0');
                gyro_x_offset_reg <= (others => '0');
                gyro_y_offset_reg <= (others => '0');
                gyro_z_offset_reg <= (others => '0');
                gyro_calib_count <= 0;                
            else
                imu_int_m    <= imu_int;
                imu_int_sync <= imu_int_m;

                case imu_state is
                    when ST_IDLE =>
                        if start_init = '1' then
                            imu_state <= ST_RESET;
                        end if;
                        
                    when ST_RESET =>
                        init_idx      <= 0;
                        byte_idx      <= 0;
                        timeout_cnt   <= 0;
                        delay_cnt     <= 0;
                        m_valid_reg   <= '0';
                        m_last_reg    <= '0';
                        ready         <= '0';
                        gyro_calib_count <= 0;
                        gyro_x_sum    <= (others => '0');
                        gyro_y_sum    <= (others => '0');
                        gyro_z_sum    <= (others => '0');
                        imu_state     <= ST_HW_RESET_CMD;

                    when ST_HW_RESET_CMD =>
                        i2c_cmd_type    <= "00";
                        i2c_reg_addr    <= x"06"; -- PWR_MGMT_1
                        i2c_wr_data     <= x"80"; -- Hardware Reset
                        i2c_cmd_valid_r <= '1';
                        timeout_cnt     <= 0; 
                        
                        if i2c_cmd_valid_r = '1' and i2c_cmd_ready = '1' then
                            i2c_cmd_valid_r <= '0';
                            imu_state       <= ST_HW_RESET_WAIT;
                        end if;

                    when ST_HW_RESET_WAIT =>
                        if timeout_cnt = TIMEOUT_CYCLES then
                            imu_state <= ST_RESET;
                        elsif i2c_done = '1' then
                            timeout_cnt <= 0;
                            imu_state   <= ST_HW_RESET_DELAY;
                        else
                            timeout_cnt <= timeout_cnt + 1;
                        end if;

                    when ST_HW_RESET_DELAY =>
                        if delay_cnt = TIMEOUT_CYCLES then
                            delay_cnt <= 0;
                            imu_state <= ST_INIT_CMD;
                        else
                            delay_cnt <= delay_cnt + 1;
                        end if;

                    when ST_INIT_CMD =>
                        if INIT_REGS(init_idx).addr = x"FF" then
                            imu_state <= ST_INIT_DELAY;
                            delay_cnt <= 0;
                        else
                            i2c_cmd_type    <= "00";
                            i2c_reg_addr    <= INIT_REGS(init_idx).addr;
                            i2c_wr_data     <= INIT_REGS(init_idx).data;
                            i2c_cmd_valid_r <= '1';
                            timeout_cnt     <= 0; 
                            
                            if i2c_cmd_valid_r = '1' and i2c_cmd_ready = '1' then
                                i2c_cmd_valid_r <= '0';
                                imu_state       <= ST_INIT_WAIT;
                            end if;
                        end if;

                    when ST_INIT_WAIT =>
                        if timeout_cnt = TIMEOUT_CYCLES then
                            imu_state <= ST_RESET;
                        elsif i2c_done = '1' then
                            timeout_cnt <= 0;
                            if init_idx = INIT_REGS'high then
                                imu_state <= ST_CALIB_WAIT_INT; -- Move to calibration
                            else
                                init_idx  <= init_idx + 1;
                                imu_state <= ST_INIT_CMD;
                            end if;
                        else
                            timeout_cnt <= timeout_cnt + 1;
                        end if;
                        
                    when ST_INIT_DELAY =>
                        -- Give 100ms delay during I2C bypass magnetometer spin up
                        if delay_cnt = TIMEOUT_CYCLES then
                            delay_cnt <= 0;
                            init_idx  <= init_idx + 1;
                            imu_state <= ST_INIT_CMD;
                        else
                            delay_cnt <= delay_cnt + 1;
                        end if;

                    --------------------------------------------------------------------------
                    -- CALIBRATION PHASE
                    --------------------------------------------------------------------------
                    when ST_CALIB_WAIT_INT =>
                        byte_idx <= 0;
                        if imu_int_sync = '1' then
                            imu_state <= ST_CALIB_BURST_CMD;
                        end if;

                    when ST_CALIB_BURST_CMD =>
                        i2c_cmd_type    <= "10";
                        i2c_reg_addr    <= x"2D"; 
                        i2c_burst_len   <= to_unsigned(22, 8); 
                        i2c_cmd_valid_r <= '1';
                        timeout_cnt     <= 0; 

                        if i2c_cmd_valid_r = '1' and i2c_cmd_ready = '1' then
                            i2c_cmd_valid_r <= '0';
                            imu_state       <= ST_CALIB_BURST_WAIT;
                        end if;

                    when ST_CALIB_BURST_WAIT =>
                        if timeout_cnt = TIMEOUT_CYCLES then
                            imu_state <= ST_RESET;
                        else
                            timeout_cnt <= timeout_cnt + 1;
                            
                            if i2c_byte_valid = '1' then
                                raw_bytes(byte_idx) <= i2c_rd_data;
                                if byte_idx < 21 then
                                    byte_idx <= byte_idx + 1;
                                end if;
                            end if;
                            
                            if i2c_done = '1' then
                                if gyro_calib_count < GYRO_CALIB_SAMPLES then
                                    gyro_x_sum <= gyro_x_sum + signed(gx_raw);
                                    gyro_y_sum <= gyro_y_sum + signed(gy_raw);
                                    gyro_z_sum <= gyro_z_sum + signed(gz_raw);
                                    gyro_calib_count <= gyro_calib_count + 1;
                                    imu_state <= ST_CALIB_WAIT_INT;
                                else
                                    gyro_x_offset_reg <= resize(gyro_x_sum / GYRO_CALIB_SAMPLES, 16);
                                    gyro_y_offset_reg <= resize(gyro_y_sum / GYRO_CALIB_SAMPLES, 16);
                                    gyro_z_offset_reg <= resize(gyro_z_sum / GYRO_CALIB_SAMPLES, 16);
                                    
                                    ready <= '1'; -- Signal that IMU is ready and calibrated
                                    imu_state <= ST_WAIT_INT; -- Move to steady stream
                                end if;
                            end if;
                        end if;

                    when ST_WAIT_INT =>
                        byte_idx <= 0;
                        if imu_int_sync = '1' then
                            imu_state <= ST_BURST_CMD;
                        end if;

                    when ST_BURST_CMD =>
                        i2c_cmd_type    <= "10";
                        i2c_reg_addr    <= x"2D"; 
                        i2c_burst_len   <= to_unsigned(22, 8); 
                        i2c_cmd_valid_r <= '1';
                        timeout_cnt     <= 0; 

                        if i2c_cmd_valid_r = '1' and i2c_cmd_ready = '1' then
                            i2c_cmd_valid_r <= '0';
                            imu_state       <= ST_BURST_WAIT;
                        end if;

                    when ST_BURST_WAIT =>
                        if timeout_cnt = TIMEOUT_CYCLES then
                            imu_state <= ST_RESET;
                        else
                            timeout_cnt <= timeout_cnt + 1;
                            
                            if i2c_byte_valid = '1' then
                                raw_bytes(byte_idx) <= i2c_rd_data;
                                if byte_idx < 21 then
                                    byte_idx <= byte_idx + 1;
                                end if;
                            end if;
                                                    
                            if i2c_done = '1' then
                                timeout_cnt  <= 0;
                                m_valid_reg  <= '1';
                                m_last_reg   <= '1'; 
                                m_axis_tkeep <= x"FFFFFFFF";
                                m_axis_tstrb <= x"FFFFFFFF";
                                                            
                                m_axis_tdata(15 downto 0)    <= ax_raw;
                                m_axis_tdata(31 downto 16)   <= ay_raw;
                                m_axis_tdata(47 downto 32)   <= az_raw;
                                m_axis_tdata(63 downto 48)   <= gx_raw;
                                m_axis_tdata(79 downto 64)   <= gy_raw;
                                m_axis_tdata(95 downto 80)   <= gz_raw;
                                m_axis_tdata(111 downto 96)  <= mx_raw;
                                m_axis_tdata(127 downto 112) <= my_raw;
                                m_axis_tdata(143 downto 128) <= mz_raw;
                                m_axis_tdata(159 downto 144) <= temp_raw;
                                
                                m_axis_tdata(175 downto 160) <= std_logic_vector(gyro_z_offset_reg);
                                m_axis_tdata(191 downto 176) <= std_logic_vector(gyro_y_offset_reg);
                                m_axis_tdata(207 downto 192) <= std_logic_vector(gyro_x_offset_reg);
                                
                                imu_state <= ST_STREAM;
                            end if;
                        end if;

                    when ST_STREAM =>
                        if m_axis_tready = '1' then
                            m_valid_reg <= '0';
                            m_last_reg  <= '0';
                            imu_state   <= ST_WAIT_INT;
                        end if;
                end case;
            end if;
        end if;
    end process;    
end architecture;