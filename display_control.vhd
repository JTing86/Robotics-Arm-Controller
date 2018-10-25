library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity display_control is
   port (
            push_button : in  std_logic_vector(1 downto 0);
             target_x    : in  std_logic_vector(3 downto 0);
             target_y    : in  std_logic_vector(3 downto 0);
             current_x   : in  std_logic_vector(3 downto 0);
             current_y   : in  std_logic_vector(3 downto 0);
             DIG1            : out std_logic_vector(3 downto 0);
             DIG2            : out std_logic_vector(3 downto 0)
        );
end entity display_control;

architecture one of display_control is

begin

    with push_button(1) select
        DIG1 <= target_x  when '1',
                 current_x when '0';

    with push_button(0) select
        DIG2 <= target_y  when '1',
                current_y when '0';

end one;
