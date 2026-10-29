library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity motor_controller is
    generic(
        CLK_FREQ             : integer := 100_000_000;
        PWM_FREQ             : integer := 400;          -- Hz 
        MIN_ALLOWED_MOTOR_US : integer := 1100;         -- 10%
        MAX_ALLOWED_MOTOR_US : integer := 1250          -- 25%
    );
    port(
        clk           : in  std_logic;
        rst           : in  std_logic;

        -- AXI Stream from mixer_module 
        s_axis_tdata  : in  std_logic_vector(127 downto 0);
        s_axis_tvalid : in  std_logic;
        s_axis_tready : out std_logic;

        -- Control signals
        start_init    : in  std_logic;
        ready         : out std_logic;
        e_stop        : in  std_logic;

        motor_pins    : out std_logic_vector(3 downto 0);
        
        debug_state   : out std_logic_vector(7 downto 0)
    );
end entity motor_controller;

architecture rtl of motor_controller is

    constant PERIOD_TICKS : integer := CLK_FREQ / PWM_FREQ; 
    
    constant TICKS_PER_US : integer := CLK_FREQ / 1_000_000;

    type state_type is (
        S_UNINIT,
        S_ARM_DELAY,
        S_IDLE,
        S_ESTOP
    );
    signal state : state_type := S_UNINIT;

    signal timer        : integer range 0 to CLK_FREQ * 5 := 0;
    
    signal m1_ticks, m2_ticks, m3_ticks, m4_ticks : integer range 0 to PERIOD_TICKS := 0;
    
    signal pwm_counter : integer range 0 to PERIOD_TICKS := 0;

    function q16_to_ticks(q16_val : std_logic_vector(31 downto 0)) return integer is
        variable val_int  : integer;
        variable us_target : integer;
        variable tick_target : integer;
    begin
        val_int := to_integer(signed(q16_val));
        
        -- Clamp input between 0.0 and 1.0 (Q16.16 format where 1.0 = 65536)
        if val_int < 0 then
            val_int := 0;
        elsif val_int > 65536 then
            val_int := 65536;
        end if;
                
        -- Map 0.0 - 1.0 to standard 1000us - 2000us ESC range
        us_target := 1000 + to_integer(shift_right(to_unsigned(val_int * 1000, 32), 16));

        -- Apply hardware safety clamps
        if us_target < MIN_ALLOWED_MOTOR_US then
            us_target := MIN_ALLOWED_MOTOR_US;
        elsif us_target > MAX_ALLOWED_MOTOR_US then
            us_target := MAX_ALLOWED_MOTOR_US;
        end if;
        
        -- Convert microseconds to clock ticks
        tick_target := us_target * TICKS_PER_US;
        return tick_target;
    end function;

begin

    ready <= '1' when state = S_IDLE else '0';
    s_axis_tready <= '1' when state = S_IDLE else '0';
    
    debug_state <= std_logic_vector(to_unsigned(state_type'pos(state), 8));

    -- Main Control & Arming FSM Process
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '0' then
                state       <= S_UNINIT;
                timer       <= 0;
                m1_ticks    <= MIN_ALLOWED_MOTOR_US * TICKS_PER_US;
                m2_ticks    <= MIN_ALLOWED_MOTOR_US * TICKS_PER_US;
                m3_ticks    <= MIN_ALLOWED_MOTOR_US * TICKS_PER_US;
                m4_ticks    <= MIN_ALLOWED_MOTOR_US * TICKS_PER_US;
            else
                -- Emergency stop overrides everything
                if e_stop = '1' then
                    state <= S_ESTOP;
                else
                    case state is
                        when S_UNINIT =>
                            if start_init = '1' then
                                -- Set motors to minimum throttle for arming pulse
                                m1_ticks <= MIN_ALLOWED_MOTOR_US * TICKS_PER_US;
                                m2_ticks <= MIN_ALLOWED_MOTOR_US * TICKS_PER_US;
                                m3_ticks <= MIN_ALLOWED_MOTOR_US * TICKS_PER_US;
                                m4_ticks <= MIN_ALLOWED_MOTOR_US * TICKS_PER_US;
                                timer <= 0;
                                state <= S_ARM_DELAY;
                            end if;

                        when S_ARM_DELAY =>
                            -- Wait 3 seconds for ESCs to initialize and arm
                            if timer >= CLK_FREQ * 3 then
                                state <= S_IDLE;
                            else
                                timer <= timer + 1;
                            end if;

                        when S_IDLE =>
                            -- Stream mode: grab incoming mixer values dynamically
                            if s_axis_tvalid = '1' then
                                m1_ticks <= q16_to_ticks(s_axis_tdata(31 downto 0));
                                m2_ticks <= q16_to_ticks(s_axis_tdata(63 downto 32));
                                m3_ticks <= q16_to_ticks(s_axis_tdata(95 downto 64));
                                m4_ticks <= q16_to_ticks(s_axis_tdata(127 downto 96));
                            end if;

                        when S_ESTOP =>
                            -- Cut signals down to 0 (or minimum safe off)
                            m1_ticks <= 0;
                            m2_ticks <= 0;
                            m3_ticks <= 0;
                            m4_ticks <= 0;
                            if e_stop = '0' then
                                state <= S_UNINIT;
                            end if;
                    end case;
                end if;
            end if;
        end if;
    end process;

    -- High-Frequency PWM Generator Process (Direct Pin Driver)
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '0' then
                pwm_counter <= 0;
                motor_pins  <= "0000";
            else
                if pwm_counter >= PERIOD_TICKS - 1 then
                    pwm_counter <= 0;
                else
                    pwm_counter <= pwm_counter + 1;
                end if;

                -- Generate PWM high/low logic for each motor pin
                if pwm_counter < m1_ticks then motor_pins(0) <= '1'; else motor_pins(0) <= '0'; end if;
                if pwm_counter < m2_ticks then motor_pins(1) <= '1'; else motor_pins(1) <= '0'; end if;
                if pwm_counter < m3_ticks then motor_pins(2) <= '1'; else motor_pins(2) <= '0'; end if;
                if pwm_counter < m4_ticks then motor_pins(3) <= '1'; else motor_pins(3) <= '0'; end if;
            end if;
        end if;
    end process;

end architecture rtl;