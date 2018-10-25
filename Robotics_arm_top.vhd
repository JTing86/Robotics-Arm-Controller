LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY LogicalStep_Lab4_top IS
   PORT
    (
   clkin_50      : in    std_logic;
    rst_n            : in    std_logic;
    pb               : in    std_logic_vector(3 downto 0);
    sw               : in  std_logic_vector(7 downto 0); -- The switch inputs
   leds          : out std_logic_vector(7 downto 0);    -- for displaying the switch content
   seg7_data     : out std_logic_vector(6 downto 0); -- 7-bit outputs to a 7-segment
    seg7_char1   : out std_logic;                           -- seg7 digi selectors
    seg7_char2   : out std_logic                            -- seg7 digi selectors
    );
END LogicalStep_Lab4_top;

ARCHITECTURE SimpleCircuit OF LogicalStep_Lab4_top IS

component Bin_Counter4bit is port (
    Main_clk     : in  std_logic := '0';
    rst_n        : in  std_logic := '0';
    clk_en       : in  std_logic := '0';
    up1_down0    : in  std_logic := '0';
    counter_bits : out std_logic_vector(3 downto 0)
);
end component;

component Bidir_shift_reg is port (
    CLK         : in  std_logic := '0';
    RESET_n     : in  std_logic := '0';
    CLK_EN      : in  std_logic := '0';
    LEFT0_RIGHT1: in  std_logic := '0';
    REG_BITS    : out std_logic_vector(3 downto 0)
);
end component;

Component Compx4 is port(
    bit_in4A     : in  std_logic_vector(3 downto 0);
    bit_in4B     : in  std_logic_vector(3 downto 0);
    bit_out      : out std_logic_vector(2 downto 0)
);
end component;

component SevenSegment is port (
   hex         :  in  std_logic_vector(3 downto 0);   -- The 4 bit data to be displayed
   sevenseg     :  out std_logic_vector(6 downto 0)    -- 7-bit outputs to a 7-segment
);
end component;

component segment7_mux is port (
    clk          : in  std_logic := '0';
    DIN2            : in  std_logic_vector(6 downto 0);
    DIN1            : in  std_logic_vector(6 downto 0);
    DOUT             : out std_logic_vector(6 downto 0);
    DIG2             : out std_logic;
    DIG1             : out std_logic;
    disable      : in std_logic
);
end component;

component display_control is port (
    push_button  : in  std_logic_vector(1 downto 0);
    target_x     : in  std_logic_vector(3 downto 0);
    target_y     : in  std_logic_vector(3 downto 0);
    current_x    : in  std_logic_vector(3 downto 0);
    current_y    : in  std_logic_vector(3 downto 0);
    DIG1             : out std_logic_vector(3 downto 0);
    DIG2             : out std_logic_vector(3 downto 0)
);
end component;

component position_control is port (
    clk_input   : IN  std_logic;
    rst_n       : IN  std_logic;
    x_en        : IN  std_logic := '0';
    y_en          : IN  std_logic := '0';
    comp_x      : IN  std_logic_vector(2 downto 0);
    comp_y      : IN  std_logic_vector(2 downto 0);
    en_mov      : IN  std_logic;
    move_x      : OUT std_logic;
    move_y      : OUT std_logic;
    up_x        : OUT std_logic;
    up_y          : OUT std_logic;
    en_ex       : OUT std_logic;
    state_error : OUT std_logic
);
end component;

component extender_control IS Port
(
    clk_input   : IN  std_logic;
    rst_n       : IN  std_logic;
    toggle_ex   : IN  std_logic;
    en_ex       : IN  std_logic;
    up_ex       : OUT std_logic;
    en_mov      : OUT std_logic;
    en_grap     : OUT std_logic
 );
END component;

component grappler_control IS Port
(
    clk_input   : IN  std_logic;
    rst_n       : IN  std_logic;
    toggle_grap : IN  std_logic;
    en_grap     : IN  std_logic;
    state_grap  : OUt std_logic
 );
END component;

----------------------------------------------------------------------------------------------------
    CONSTANT    SIM                         :  boolean := FALSE;    -- set to TRUE for simulation runs otherwise keep at 0.
   CONSTANT CLK_DIV_SIZE                :   INTEGER := 26;    -- size of vectors for the counters

   SIGNAL   Main_CLK                        :  STD_LOGIC;           -- main clock to drive sequencing of State Machine

    SIGNAL  bin_counter                 :  UNSIGNED(CLK_DIV_SIZE-1 downto 0); -- := to_unsigned(0,CLK_DIV_SIZE); -- reset binary counter to zero

    SIGNAL   digit_1                 : std_logic_vector(3 downto 0); -- 4 bit data for digit 2 (x)
    SIGNAL   digit_2                 : std_logic_vector(3 downto 0); -- 4 bit data for digit 2 (y)
    SIGNAL   hex_1                   : std_logic_vector(6 downto 0); -- 7 bit for seven segment digit 1 (x)
    SIGNAL   hex_2                   : std_logic_vector(6 downto 0); -- 7 bit for seven segment digit 2 (y)

    SIGNAL   tcurrent_x              : std_logic_vector(3 downto 0) := "0000"; -- current position x
    SIGNAL   tcurrent_y              : std_logic_vector(3 downto 0) := "0000"; -- current position y

    SIGNAL   target_x                : std_logic_vector(3 downto 0); -- target position x
    SIGNAL   target_y                : std_logic_vector(3 downto 0); -- target position y

    signal   comp_x                  : std_logic_vector(2 downto 0) := "000"; -- comparison x (greater, equal, lower)
    signal   comp_y                  : std_logic_vector(2 downto 0) := "000"; -- comparison y (greater, equal, lower)

    signal   tmove_x                 : std_logic := '0'; -- enable clock x for bit counter
    signal   tmove_y                 : std_logic := '0'; -- enable clock y for bit counter
    signal   tup_x                   : std_logic; -- up = 1 down = 0 (up1down0) (x)
    signal   tup_y                   : std_logic; -- up = 1 down = 0 (up1down0) (y)

    signal   ten_ex                  : std_logic; -- extender enable
    signal   ten_mov                 : std_logic := '1'; -- extender out / enable move

    signal   ten_grap                : std_logic; -- enable grappler

    signal   tup_ex                  : std_logic; -- right = 1 left = 0 (right1left0) leds(7 downto 4)

    signal   tcurrent_ex             : std_logic_vector(3 downto 0) := "0000"; -- current extender state leds(7 downto 4)

    signal   tstate_grap             : std_logic; -- current grappler state leds(3)

    signal   tstate_error            : std_logic; -- current error state leds(0)

    signal   flashed                 : std_logic; -- flashed ..

----------------------------------------------------------------------------------------------------
BEGIN

-- CLOCKING GENERATOR WHICH DIVIDES THE INPUT CLOCK DOWN TO A LOWER FREQUENCY

BinCLK: PROCESS(clkin_50, rst_n) is
   BEGIN
        IF (rising_edge(clkin_50)) THEN -- binary counter increments on rising clock edge
         bin_counter <= bin_counter + 1;
      END IF;
   END PROCESS;

Clock_Source:
                Main_Clk <=
                clkin_50 when sim = TRUE else               -- for simulations only
                std_logic(bin_counter(23));                             -- for real FPGA operation

-- process that controls the flashing of the seven segment display
flashing: PROCESS (Main_CLK, tstate_error) is
BEGIN
    IF (rising_edge(Main_CLK)) THEN
        IF (tstate_error = '1') THEN
            flashed <= NOT flashed;
        ELSE
            flashed <= '0';
        END IF;
    END IF;
END PROCESS;

-- read the target position from the switches
target_x <= sw(7 downto 4);
target_y <= sw(3 downto 0);

-- multiplexor between current and target position
INST1 : display_control  port map (pb(3 downto 2), target_x, target_y, tcurrent_x, tcurrent_y, digit_1, digit_2);

-- Encoding 4 bits into 7 bits for seven segments
INST2 : SevenSegment     port map (digit_1, hex_1);
INST3 : SevenSegment     port map (digit_2, hex_2);

-- display proper digit given 7 bits. Added functionality: flashed which disables the display for erro states
INST4 : segment7_mux     port map (clkin_50, hex_2, hex_1, seg7_data, seg7_char2, seg7_char1, flashed);

-- compare current and target position for both x and y
INST5 : Compx4           port map (tcurrent_x, target_x, comp_x);
INST6 : Compx4           port map (tcurrent_y, target_y, comp_y);

-- state machine controlling x and y (mealy)
INST9 : position_control port map (Main_CLK, rst_n, pb(3), pb(2), comp_x, comp_y,ten_mov, tmove_x, tmove_y, tup_x, tup_y, ten_ex, tstate_error);

-- from position_control given the state and inputs, it counts up or down for x and y
INST8 : Bin_Counter4bit  port map (Main_CLK, rst_n, tmove_x, tup_x, tcurrent_x);
INST7 : Bin_Counter4bit  port map (Main_CLK, rst_n, tmove_y, tup_y, tcurrent_y);

-- state machine controlling the extender leds(7 downto 4) (moore)
INST10: extender_control port map (Main_CLK, rst_n, pb(1), ten_ex, tup_ex, ten_mov, ten_grap);

-- from extender_control given the state, its shifts left or right for leds(7 downto 4)
INST11: Bidir_shift_reg  port map (Main_CLK, rst_n, '1', tup_ex, tcurrent_ex);

-- state machine controlling the grappler leds(3) (moore)
INST12: grappler_control port map (Main_CLK, rst_n, pb(0), ten_grap, tstate_grap);

-- concatenation of the leds for output
leds <= tcurrent_ex & tstate_grap & "00" & tstate_error;

---------------------------------------------------------------------------------------------------

END SimpleCircuit;
