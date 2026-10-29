library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pca9685_controller is
    generic(
        CLK_FREQ : integer := 100_000_000;
        MAX_ALLOWED_MOTOR_US : integer := 1250; -- 25%
        MIN_ALLOWED_MOTOR_US : integer := 1100  -- 10 %
    );
    port(
        clk           : in  std_logic;
        rst           : in  std_logic;

        -- AXI Stream from mixer_module
        s_axis_tdata  : in  std_logic_vector(127 downto 0);
        s_axis_tvalid : in  std_logic;
        s_axis_tready : out std_logic;

        -- Control signals
        start_init     : in  std_logic;
        ready          : out std_logic;

        -- Separated I2C Tristate control
        scl_i         : in  std_logic;
        scl_t         : out std_logic;
        sda_i         : in  std_logic;
        sda_t         : out std_logic;
        
        
        e_stop : in std_logic;
        
        debug_state : out std_logic_vector(7 downto 0)
    );
end entity pca9685_controller;

architecture rtl of pca9685_controller is

    component i2c_master is
        generic(
            CLK_FREQ : integer := 100_000_000;
            I2C_FREQ : integer := 400_000
        );
        port(
            clk         : in  std_logic;
            rst         : in  std_logic;
            cmd_valid   : in  std_logic;
            cmd_ready   : out std_logic;
            cmd_type    : in  std_logic_vector(1 downto 0);
            dev_addr    : in  std_logic_vector(6 downto 0);
            reg_addr    : in  std_logic_vector(7 downto 0);
            wr_data     : in  std_logic_vector(7 downto 0);
            rd_data     : out std_logic_vector(7 downto 0);
            burst_len   : in  unsigned(7 downto 0);
            byte_valid  : out std_logic;
            done        : out std_logic;
            busy        : out std_logic;
            error       : out std_logic;
            scl_i       : in  std_logic;
            scl_t       : out std_logic;
            sda_i       : in  std_logic;
            sda_t       : out std_logic
        );
    end component;

    type state_type is (
        S_UNINIT, S_IDLE,
        S_INIT_1, S_INIT_2, S_INIT_3, S_INIT_4, S_INIT_5,
        S_INIT_6, S_INIT_7, S_INIT_8, S_INIT_9, S_INIT_10, S_PRE_ARM_DELAY,
        S_ARM_1, S_ARM_2, S_ARM_3,
        S_MOTOR_WRITE_START, S_MW_ON_L, S_MW_ON_H, S_MW_OFF_L, S_MW_OFF_H, S_MW_NEXT,
        I2C_SEND, I2C_WAIT_BUSY, I2C_WAIT_DONE,
        DELAY_START, DELAY_WAIT,
        S_ESTOP_SEND, S_ESTOP_HALT
    );
    signal state, return_state, mw_return_state : state_type := S_UNINIT;

    signal timer        : integer := 0;
    signal delay_target : integer := 0;
    
    signal i2c_rst        : std_logic;    
    
    -- Motor variables
    signal m1_pwm, m2_pwm, m3_pwm, m4_pwm : integer := 0;
    signal m_idx         : integer range 1 to 4 := 1;
    signal m_val_current : integer := 0;
    signal m_base_reg    : std_logic_vector(7 downto 0) := x"00";
    signal is_stream_mode: std_logic := '0';

    -- I2C Master signals
    signal i2c_cmd_valid : std_logic := '0';
    signal i2c_cmd_ready : std_logic;
    signal i2c_reg_addr  : std_logic_vector(7 downto 0) := (others => '0');
    signal i2c_wr_data   : std_logic_vector(7 downto 0) := (others => '0');
    signal i2c_done      : std_logic;
    signal i2c_busy_out  : std_logic;

    constant DEV_ADDR : std_logic_vector(6 downto 0) := "1000000"; -- 0x40
    
    -- Used only during ESC Arming sequence
    constant MIN_VAL  : integer := 1638; -- ~1000us off time at 400Hz
    constant MAX_VAL  : integer := 3277; -- ~2000us off time at 400Hz

    -- Fixed PWM Calculation function
    function calc_pwm(q16_val : std_logic_vector(31 downto 0)) return integer is
        variable val_int    : integer;
        variable us_target  : integer;
        variable res        : integer;
    begin
        val_int := to_integer(signed(q16_val));
        
        -- Clamp input to 0.0 - 1.0 (Q16.16)
        if val_int < 0 then
            val_int := 0;
        elsif val_int > 65536 then
            val_int := 65536;
        end if;
                
        -- Map 0.0 - 1.0 directly to the standard 1000us - 2000us ESC range
        -- (2000 - 1000 = 1000)
        us_target := 1000 + to_integer(shift_right(to_unsigned(val_int * 1000, 32), 16));

        -- Hardware safety clamp overriding the mapped target
        if us_target < MIN_ALLOWED_MOTOR_US then
            us_target := MIN_ALLOWED_MOTOR_US;
        elsif us_target > MAX_ALLOWED_MOTOR_US then
            us_target := MAX_ALLOWED_MOTOR_US;
        end if;
        
        -- Convert microsecond target to PCA9685 12-bit format
        res := to_integer(shift_right(to_unsigned(us_target * 53687, 32), 15));
        return res;
    end function;
    
begin
    i2c_rst <= not rst;
    
    -- Output flags are ONLY high in S_IDLE (After initialization is complete)
    ready <= '1' when state = S_IDLE else '0';
    s_axis_tready <= '1' when state = S_IDLE else '0';
    
    debug_state <= std_logic_vector(to_unsigned(state_type'pos(state), 8));

    i2c_inst : i2c_master
    generic map(
        CLK_FREQ => CLK_FREQ,
        I2C_FREQ => 400_000
    )
    port map(
        clk        => clk,
        rst        => i2c_rst,
        cmd_valid  => i2c_cmd_valid,
        cmd_ready  => i2c_cmd_ready,
        cmd_type   => "00",      
        dev_addr   => DEV_ADDR,
        reg_addr   => i2c_reg_addr,
        wr_data    => i2c_wr_data,
        rd_data    => open,
        burst_len  => x"00",     
        byte_valid => open,
        done       => i2c_done,
        busy       => i2c_busy_out,
        error      => open,
        scl_i      => scl_i,
        scl_t      => scl_t,
        sda_i      => sda_i,
        sda_t      => sda_t
    );

    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '0' then
                state <= S_UNINIT;
                i2c_cmd_valid <= '0';
            else
                case state is
                    
                    when S_UNINIT =>
                        if start_init = '1' then
                            state <= S_INIT_1;
                        end if;

                    when S_IDLE =>
                        if e_stop = '1' then
                            state <= S_ESTOP_SEND;
                        elsif s_axis_tvalid = '1' then
                            m1_pwm <= calc_pwm(s_axis_tdata(31 downto 0));
                            m2_pwm <= calc_pwm(s_axis_tdata(63 downto 32));
                            m3_pwm <= calc_pwm(s_axis_tdata(95 downto 64));
                            m4_pwm <= calc_pwm(s_axis_tdata(127 downto 96));
                            
                            m_idx <= 1;
                            m_val_current <= calc_pwm(s_axis_tdata(31 downto 0));
                            is_stream_mode <= '1';
                            mw_return_state <= S_IDLE;
                            state <= S_MOTOR_WRITE_START;
                        end if;

                    -- =======================================================
                    -- Initialization Phase 
                    -- =======================================================
                    when S_INIT_1 => 
                        i2c_reg_addr <= x"01"; i2c_wr_data <= x"04"; 
                        return_state <= S_INIT_2; state <= I2C_SEND;
                    when S_INIT_2 => 
                        i2c_reg_addr <= x"00"; i2c_wr_data <= x"01"; 
                        return_state <= S_INIT_3; state <= I2C_SEND;
                    when S_INIT_3 => 
                        delay_target <= CLK_FREQ / 200; 
                        return_state <= S_INIT_4; state <= DELAY_START;
                    when S_INIT_4 => 
                        i2c_reg_addr <= x"00"; i2c_wr_data <= x"01"; 
                        return_state <= S_INIT_5; state <= I2C_SEND;
                    when S_INIT_5 => 
                        delay_target <= CLK_FREQ / 200; 
                        return_state <= S_INIT_6; state <= DELAY_START;
                    when S_INIT_6 => 
                        i2c_reg_addr <= x"00"; i2c_wr_data <= x"11"; 
                        return_state <= S_INIT_7; state <= I2C_SEND;
                    when S_INIT_7 => 
                        i2c_reg_addr <= x"FE"; i2c_wr_data <= x"0E"; 
                        return_state <= S_INIT_8; state <= I2C_SEND;
                    when S_INIT_8 => 
                        i2c_reg_addr <= x"00"; i2c_wr_data <= x"01"; 
                        return_state <= S_INIT_9; state <= I2C_SEND;
                    when S_INIT_9 => 
                        delay_target <= CLK_FREQ / 200; 
                        return_state <= S_INIT_10; state <= DELAY_START;
                    when S_INIT_10 =>
                        i2c_reg_addr <= x"00"; i2c_wr_data <= x"81";
                        return_state <= S_PRE_ARM_DELAY; state <= I2C_SEND;
                                            
                    when S_PRE_ARM_DELAY =>
                        delay_target <= CLK_FREQ;
                        m_idx <= 1; -- Initialize arming index for motor 1
                        return_state <= S_ARM_1; 
                        state <= DELAY_START;

                    -- =======================================================
                    -- Sequential Arming Phase (3 seconds per motor)
                    -- =======================================================
                    when S_ARM_1 =>
                        m_val_current <= MIN_VAL; 
                        is_stream_mode <= '0';
                        mw_return_state <= S_ARM_2; 
                        state <= S_MOTOR_WRITE_START;
                        
                    when S_ARM_2 =>
                        delay_target <= CLK_FREQ * 3; -- 3000ms delay for ESC safety check
                        return_state <= S_ARM_3; 
                        state <= DELAY_START;
                        
                    when S_ARM_3 =>
                        if m_idx = 4 then
                            state <= S_IDLE; -- All motors sequentially armed
                        else
                            m_idx <= m_idx + 1;
                            state <= S_ARM_1; -- Loop to arm next motor
                        end if;

                    -- =======================================================
                    -- Iterative Motor Registry Writing Sub-FSM
                    -- =======================================================
                    when S_MOTOR_WRITE_START =>
                        if m_idx = 1 then m_base_reg <= x"12";
                        elsif m_idx = 2 then m_base_reg <= x"16";
                        elsif m_idx = 3 then m_base_reg <= x"22";
                        elsif m_idx = 4 then m_base_reg <= x"26";
                        end if;
                        state <= S_MW_ON_L;
                        
                    when S_MW_ON_L =>
                        i2c_reg_addr <= m_base_reg;
                        i2c_wr_data <= x"00";
                        return_state <= S_MW_ON_H;
                        state <= I2C_SEND;
                        
                    when S_MW_ON_H =>
                        i2c_reg_addr <= std_logic_vector(unsigned(m_base_reg) + 1);
                        i2c_wr_data <= x"00";
                        return_state <= S_MW_OFF_L;
                        state <= I2C_SEND;
                        
                    when S_MW_OFF_L =>
                        i2c_reg_addr <= std_logic_vector(unsigned(m_base_reg) + 2);
                        i2c_wr_data <= std_logic_vector(to_unsigned(m_val_current, 16)(7 downto 0));
                        return_state <= S_MW_OFF_H;
                        state <= I2C_SEND;
                        
                    when S_MW_OFF_H =>
                        i2c_reg_addr <= std_logic_vector(unsigned(m_base_reg) + 3);
                        i2c_wr_data <= std_logic_vector(to_unsigned(m_val_current, 16)(15 downto 8));
                        return_state <= S_MW_NEXT;
                        state <= I2C_SEND;
                        
                    when S_MW_NEXT =>
                        if is_stream_mode = '0' then
                            -- Single-motor write mode for sequential arming
                            state <= mw_return_state;
                        else
                            -- Burst-write mode for AXI stream updates
                            if m_idx = 4 then
                                state <= mw_return_state;
                            else
                                m_idx <= m_idx + 1;
                                if m_idx = 1 then m_val_current <= m2_pwm;
                                elsif m_idx = 2 then m_val_current <= m3_pwm;
                                elsif m_idx = 3 then m_val_current <= m4_pwm;
                                end if;
                                state <= S_MOTOR_WRITE_START;
                            end if;
                        end if;

                    -- =======================================================
                    -- I2C Command Driver Sub-FSM
                    -- =======================================================
                    when I2C_SEND =>
                        i2c_cmd_valid <= '1';
                        if i2c_cmd_valid = '1' and i2c_cmd_ready = '1' then
                            i2c_cmd_valid <= '0';
                            timer <= 0; 
                            state <= I2C_WAIT_DONE;
                        end if;
                                                                                                        
                    when I2C_WAIT_BUSY =>
                        i2c_cmd_valid <= '0';
                        if i2c_busy_out = '1' then
                            state <= I2C_WAIT_DONE;
                        end if;
                        
                    when I2C_WAIT_DONE =>
                        if timer >= (CLK_FREQ / 10) then 
                            state <= S_UNINIT; 
                        elsif i2c_done = '1' then
                            state <= return_state;
                        else
                            timer <= timer + 1;
                        end if;
                                        
                    -- =======================================================
                    -- System Timer Delay Sub-FSM
                    -- =======================================================
                    when DELAY_START =>
                        timer <= 0;
                        state <= DELAY_WAIT;
                        
                    when DELAY_WAIT =>
                        if timer >= delay_target then
                            state <= return_state;
                        else
                            timer <= timer + 1;
                        end if;
                        
                    when S_ESTOP_SEND =>
                        i2c_reg_addr <= x"FD"; -- PCA9685 ALL_LED_OFF_H register
                        i2c_wr_data  <= x"10"; -- Set Bit 4 (LED Full OFF bit) to instantly kill all PWM
                        return_state <= S_ESTOP_HALT; 
                        state <= I2C_SEND;
                    
                    when S_ESTOP_HALT =>
                        if e_stop = '0' then
                            state <= S_UNINIT; 
                        end if;
                                                                
                end case;
            end if;
        end if;
    end process;
end architecture rtl;