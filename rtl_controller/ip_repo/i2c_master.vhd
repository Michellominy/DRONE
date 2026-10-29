library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity i2c_master is
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

        -- Tristate Interface
        scl_i       : in  std_logic;
        scl_t       : out std_logic;
        sda_i       : in  std_logic;
        sda_t       : out std_logic

    );
end entity;

architecture rtl of i2c_master is

    constant QUARTER_PERIOD : integer := CLK_FREQ / (I2C_FREQ * 4);

    type state_t is (
        IDLE, START, CHK_PHASE,
        WRITE_BIT, GET_ACK,
        RESTART,
        READ_BIT, SEND_ACK,
        STOP
    );
    signal state : state_t := IDLE;

    type phase_t is (P_DEV_ADDR_W, P_REG_ADDR, P_WRITE_DATA, P_RESTART, P_DEV_ADDR_R, P_READ_DATA, P_STOP);
    signal phase_state : phase_t := P_DEV_ADDR_W;

    signal tick_cnt  : integer range 0 to QUARTER_PERIOD := 0;
    signal i2c_tick  : std_logic := '0';

    -- Command Buffer (Latched)
    signal cmd_ready_reg : std_logic := '1';
    signal cmd_latched   : std_logic := '0';
    signal dev_addr_reg  : std_logic_vector(6 downto 0);
    signal reg_addr_reg  : std_logic_vector(7 downto 0);
    signal wr_data_reg   : std_logic_vector(7 downto 0);
    signal cmd_type_reg  : std_logic_vector(1 downto 0);
    signal burst_len_reg : unsigned(7 downto 0);

    -- I2C Physical Core
    signal scl_out  : std_logic := '1';
    signal sda_out  : std_logic := '1';
    signal q_cnt    : integer range 0 to 3 := 0;
    signal bit_idx  : integer range 0 to 7 := 0;
    signal byte_cnt : unsigned(7 downto 0) := (others => '0');
    signal shift_reg: std_logic_vector(7 downto 0) := (others => '0');

    -- Status registers
    signal busy_reg  : std_logic := '0';
    signal error_reg : std_logic := '0';

begin

    -- Drive external tristate control signals ('0' to drive low, '1' to release)
    scl_t <= scl_out;
    sda_t <= sda_out;

    cmd_ready <= cmd_ready_reg;
    busy      <= busy_reg;
    error     <= error_reg;

    ----------------------------------------------------------------------------
    -- Unified Synchronous Process (Fixes MDRV-1 Multiple Drivers)
    ----------------------------------------------------------------------------
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                tick_cnt      <= 0;
                i2c_tick      <= '0';
                cmd_ready_reg <= '1';
                cmd_latched   <= '0';
                state         <= IDLE;
                phase_state   <= P_DEV_ADDR_W;
                scl_out       <= '1';
                sda_out       <= '1';
                q_cnt         <= 0;
                busy_reg      <= '0';
                error_reg     <= '0';
                done          <= '0';
                byte_valid    <= '0';
            else
                -- 1. Tick Generator
                if tick_cnt = QUARTER_PERIOD - 1 then
                    tick_cnt <= 0;
                    i2c_tick <= '1';
                else
                    tick_cnt <= tick_cnt + 1;
                    i2c_tick <= '0';
                end if;

                -- Default single-cycle pulses
                done       <= '0';
                byte_valid <= '0';

                -- 2. Handshake Capture (Happens at clk speed)
                if cmd_valid = '1' and cmd_ready_reg = '1' then
                    dev_addr_reg  <= dev_addr;
                    reg_addr_reg  <= reg_addr;
                    wr_data_reg   <= wr_data;
                    cmd_type_reg  <= cmd_type;
                    burst_len_reg <= burst_len;
                    
                    cmd_ready_reg <= '0';
                    cmd_latched   <= '1';
                end if;

                -- 3. Phase Generator & FSM (Happens at I2C tick speed)
                if i2c_tick = '1' then
                    case state is
                        when IDLE =>
                            busy_reg <= '0';
                            scl_out  <= '1';
                            sda_out  <= '1';
                            q_cnt    <= 0;

                            if cmd_latched = '1' then
                                cmd_latched <= '0'; -- Driven only by this process! Safe.
                                busy_reg    <= '1';
                                error_reg   <= '0';
                                phase_state <= P_DEV_ADDR_W;
                                state       <= START;
                            end if;

                        when START =>
                            if q_cnt = 0 then
                                sda_out <= '1'; scl_out <= '1'; q_cnt <= 1;
                            elsif q_cnt = 1 then
                                sda_out <= '0'; scl_out <= '1'; q_cnt <= 2;
                            elsif q_cnt = 2 then
                                sda_out <= '0'; scl_out <= '0'; q_cnt <= 3;
                            else
                                q_cnt <= 0;
                                state <= CHK_PHASE;
                            end if;

                        when CHK_PHASE =>
                            case phase_state is
                                when P_DEV_ADDR_W =>
                                    shift_reg <= dev_addr_reg & '0';
                                    bit_idx <= 7; state <= WRITE_BIT;
                                when P_REG_ADDR =>
                                    shift_reg <= reg_addr_reg;
                                    bit_idx <= 7; state <= WRITE_BIT;
                                when P_WRITE_DATA =>
                                    shift_reg <= wr_data_reg;
                                    bit_idx <= 7; state <= WRITE_BIT;
                                when P_RESTART =>
                                    state <= RESTART;
                                when P_DEV_ADDR_R =>
                                    shift_reg <= dev_addr_reg & '1';
                                    bit_idx <= 7; state <= WRITE_BIT;
                                when P_READ_DATA =>
                                    bit_idx <= 7; state <= READ_BIT;
                                when P_STOP =>
                                    state <= STOP;
                            end case;

                        when WRITE_BIT =>
                            if q_cnt = 0 then
                                scl_out <= '0'; sda_out <= shift_reg(bit_idx); q_cnt <= 1;
                            elsif q_cnt = 1 then
                                scl_out <= '1'; q_cnt <= 2;
                            elsif q_cnt = 2 then
                                scl_out <= '1'; q_cnt <= 3;
                            else
                                scl_out <= '0'; q_cnt <= 0;
                                if bit_idx = 0 then
                                    state <= GET_ACK;
                                else
                                    bit_idx <= bit_idx - 1;
                                end if;
                            end if;

                        when GET_ACK =>
                            if q_cnt = 0 then
                                scl_out <= '0'; sda_out <= '1'; q_cnt <= 1;
                            elsif q_cnt = 1 then
                                scl_out <= '1'; q_cnt <= 2;
                            elsif q_cnt = 2 then
                                scl_out <= '1'; 
                                -- Replaced 'sda' with 'sda_i'
                                if sda_i /= '0' then error_reg <= '1'; end if;
                                q_cnt <= 3;
                            else
                                scl_out <= '0'; q_cnt <= 0;
                                case phase_state is
                                    when P_DEV_ADDR_W => phase_state <= P_REG_ADDR; state <= CHK_PHASE;
                                    when P_REG_ADDR =>
                                        if cmd_type_reg = "00" then phase_state <= P_WRITE_DATA;
                                        else phase_state <= P_RESTART; end if;
                                        state <= CHK_PHASE;
                                    when P_WRITE_DATA => phase_state <= P_STOP; state <= CHK_PHASE;
                                    when P_DEV_ADDR_R => 
                                        phase_state <= P_READ_DATA; byte_cnt <= (others => '0'); state <= CHK_PHASE;
                                    when others => state <= STOP;
                                end case;
                            end if;

                        when RESTART =>
                            if q_cnt = 0 then sda_out <= '1'; scl_out <= '0'; q_cnt <= 1;
                            elsif q_cnt = 1 then scl_out <= '1'; sda_out <= '1'; q_cnt <= 2;
                            elsif q_cnt = 2 then sda_out <= '0'; scl_out <= '1'; q_cnt <= 3;
                            else scl_out <= '0'; q_cnt <= 0; phase_state <= P_DEV_ADDR_R; state <= CHK_PHASE;
                            end if;

                        when READ_BIT =>
                            if q_cnt = 0 then scl_out <= '0'; sda_out <= '1'; q_cnt <= 1;
                            elsif q_cnt = 1 then scl_out <= '1'; q_cnt <= 2;
                            -- Replaced 'sda' with 'sda_i'
                            elsif q_cnt = 2 then scl_out <= '1'; shift_reg(bit_idx) <= sda_i; q_cnt <= 3;
                            else
                                scl_out <= '0'; q_cnt <= 0;
                                if bit_idx = 0 then state <= SEND_ACK;
                                else bit_idx <= bit_idx - 1; end if;
                            end if;

                        when SEND_ACK =>
                            if q_cnt = 0 then
                                scl_out <= '0';
                                if cmd_type_reg = "01" then sda_out <= '1'; -- Single Read NACK
                                elsif byte_cnt < burst_len_reg - 1 then sda_out <= '0'; -- Burst ACK
                                else sda_out <= '1'; end if; -- Burst End NACK
                                q_cnt <= 1;
                            elsif q_cnt = 1 then
                                scl_out <= '1'; q_cnt <= 2;
                            elsif q_cnt = 2 then
                                scl_out <= '1'; rd_data <= shift_reg; byte_valid <= '1'; q_cnt <= 3;
                            else
                                scl_out <= '0'; q_cnt <= 0;
                                if cmd_type_reg = "01" then phase_state <= P_STOP; state <= CHK_PHASE;
                                else
                                    if byte_cnt < burst_len_reg - 1 then
                                        byte_cnt <= byte_cnt + 1; bit_idx <= 7; state <= READ_BIT;
                                    else phase_state <= P_STOP; state <= CHK_PHASE; end if;
                                end if;
                            end if;

                        when STOP =>
                            if q_cnt = 0 then scl_out <= '0'; sda_out <= '0'; q_cnt <= 1;
                            elsif q_cnt = 1 then scl_out <= '1'; sda_out <= '0'; q_cnt <= 2;
                            elsif q_cnt = 2 then scl_out <= '1'; sda_out <= '1'; q_cnt <= 3;
                            else
                                q_cnt <= 0;
                                done <= '1';
                                cmd_ready_reg <= '1'; -- Release bus back to user
                                state <= IDLE;
                            end if;
                    end case;
                end if;
            end if;
        end if;
    end process;


end architecture;