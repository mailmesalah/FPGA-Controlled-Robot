
--high level description of robot behaviour 
library ieee; 
use ieee.std_logic_1164.all; 
 

entity logic is 
  port (
        reset, clock, sound, bump, light, lighton, soundon : in std_logic; 
        left,right: out std_logic; 
        leftpulse,rightpulse: inout std_logic; 
        motor: inout std_logic_vector(1 downto 0)  -- new. 
  ); 
end logic; 
 

architecture behaviour of logic is
component motorcontrol 
 port(
      reset, clock: in std_logic;      --clock signal 
      next_state: in std_logic_vector(1 downto 0); --logic for motor 
      left,right: out std_logic;      --forward/reverse 
      leftpulse,rightpulse: inout std_logic    --pwm output for each motor 
 ); 
end component;

for all: motorcontrol use entity work.motorcontrol(behaviour);

signal motor_internal: std_logic_vector( 1 downto 0); 
--signal motor: std_logic_vector(1 downto 0);

BEGIN 
  comb_logic : PROCESS(sound,soundon,light,lighton,bump) 
    BEGIN 
      CASE bump IS 
        WHEN '0' =>          -- NO BUMP 
          motor_internal <= "11";       -- move forward

          CASE lighton IS 
            WHEN '0' =>         -- NOT looking for LIGHT (selected by USER) 
            --motor_internal <= "11";      -- move forward

            CASE soundon IS 
              WHEN '1' =>        -- SEEKING sound (selected by USER) 
  
              CASE sound IS 
                WHEN '1' =>       -- FOUND sound 
                  motor_internal <= "01";    -- turn Counter-clockwise 
                WHEN OTHERS =>      -- NO sound 
                  motor_internal <= "11";     -- move forward 
              END CASE;   -- END of CASE for SOUND

              WHEN OTHERS =>       -- NOT SEEKING sound (selected by USER) 
                motor_internal <= "11";       -- move forward by default. 
            END CASE;   -- END of CASE for SOUNDON.
        
            WHEN '1' => 
            CASE light IS 
                WHEN '0' =>        -- NO light found. 
     motor_internal <= "11";     -- move forward 
      WHEN '1' =>        -- FOUND light 
       motor_internal <= "10";     -- turn Clockwise (ie. towards the light) 
      WHEN OTHERS => 
       motor_internal <= "11";     -- move forward by default. 
    END CASE;   -- END of CASE for LIGHT.

     WHEN OTHERS => 
    motor_internal <= "11";     -- move forward by LIGHTON default 
   END CASE;    -- END of CASE for LIGHTON 
  
    WHEN OTHERS =>         -- BUMP is ON or UNKNOWN (just to be safe) 
   motor_internal <= "01";       -- turn Counter-clockwise to AVOID object or edge. 
  END CASE;     -- END of CASE for BUMPON

-- 
END PROCESS comb_logic; 
-- 
motor_output: PROCESS(reset,clock) 
BEGIN 
-- 
 IF reset='1' THEN 
  motor <="11";   -- manual reset 
 ELSIF rising_edge(clock) THEN 
  motor <= motor_internal; -- send output to motor 
 END IF; 
-- 
END PROCESS motor_output;

pm: motorcontrol PORT MAP(reset,clock, motor,left,right,leftpulse,rightpulse); 
 -- connect to motorcontrol routine 
  
END behaviour;

  
-- motorcontrol: interprets control logic and intitiates motor control 
-- and turning subroutines

LIBRARY ieee; 
USE ieee.std_logic_1164.ALL;

ENTITY motorcontrol IS 
 PORT(clock,reset: IN std_logic;       --clock signal and reset 
      next_state: IN std_logic_vector(1 downto 0);

      left,right: OUT std_logic;       --forward/reverse 
      leftpulse: INOUT std_logic; 
      rightpulse: INOUT std_logic     --pwm output for each motor 
 ); 
END motorcontrol;

ARCHITECTURE behaviour OF motorcontrol IS

BEGIN

logic : PROCESS(clock,next_state,reset) --H bridge forward/reverse 
        --left right outputs 
VARIABLE turncount, turntime,turnon: INTEGER; 
VARIABLE motor_input:      STD_LOGIC_VECTOR(1 DOWNTO 0);

  --turncount-counter to compare to 
BEGIN 
 turntime := 3750;    -- on a 2500 Hz clock this is 1/5 seconds 
 IF (rising_edge(clock)) THEN 
  IF (reset='1') THEN 
   turncount :=0; 
   turnon:=0; 
   motor_input := next_state; 
   left <= '1'; 
   right <= '1'; 
  END IF; 
  
-- when motor_input=00 it drives forward, motor_input=01 it turns left, motor_input=10 it turns right 
-- by default any undefined case, drives forward 
-- left and right signals connect to SGS-Thomson Input pins (1=forward, 0 = reverse), 
-- the Enable pins on the SGS-Thomson is for the PWM signal 
   IF (turnon=0) THEN 
    motor_input:=next_state; 
   END IF; 
   IF (motor_input="11" and turnon=0) THEN 
    left<='1'; 
    right<='1'; 
    motor_input := next_state; 
  
   -- AVOID A BUMP OR TABLE EDGE!  (OR TURN DUE TO SOUND) 
   ELSIF ((motor_input ="01") and (turncount < turntime)) THEN 
    turnon:=1; 
    IF (turncount < 1875) THEN  -- lasts .75 seconds on 2500 Hz Clock 
     turncount:=turncount+1; 
     left<='0'; 
     right<='0'; 
    ELSE 
     turncount:=turncount+1; 
     left<='0'; 
     right<='1'; 
    END IF; 
  
   -- TAG!! YOU'RE IT!  ROBOT SHOULD SPIN AROUND WHEN "TAGGED" BY HUMAN. 
   ELSIF (motor_input="10") THEN 
    turnon:=1; 
    IF (turncount < turntime) THEN 
     turncount:=turncount+1; 
     left<='1'; 
     right<='0'; 
    ELSE 
     turncount :=0; 
     turnon := 0; 
     motor_input := next_state; 
    END IF; 
  
   ELSE 
    turncount:=0; 
    turnon:=0; 
    motor_input:=next_state; 
  
   --ELSE 
   -- turncount:=0; 
   -- turnon:=0; 
   -- motor_input:=next_state; 
   --ELSIF (motor_input="10") THEN 
   -- turnon:=1; 
   -- IF (turncount < turntime) THEN 
   --  turncount:=turncount+1; 
   --  left<='1'; 
   --  right<='0'; 
   -- ELSE 
   --  turncount :=0; 
   --  turnon:=0; 
   --  motor_input := next_state; 
   -- END IF;

  END IF; 
 END IF; 
 

END process logic; 
  
  
--Motor Speed Feedback Control in VHDL: 
-- 
-- this process will count the shaft encoder feedback pulses. 
-- another process will sample the result of this process 
-- 
--count_shaft_feedback: PROCESS (reset, rightfeedback, leftfeedback, reset_shaft_counters) 
-- variable countleft, countright: integer; 
--BEGIN 
--   CASE reset IS 
--  WHEN '1' => 
--    countleft := 0; 
--    countright := 0; 
--  WHEN OTHERS => 
-- 
--    CASE reset_shaft_counters IS 
--   WHEN '1' => 
--     countleft := 0; 
--     countright := 0; 
--   WHEN OTHERS => 
--     IF (rising_edge(leftfeedback)) THEN 
--    countleft := countleft + 1; 
--     ELSE 
--    countleft := countleft; 
--     END IF; 
-- 
--     IF (rising_edge(rightfeedback)) THEN 
--    countright := countright +1; 
--     ELSE 
--    countright := countright; 
--     END IF; 
-- 
--    END CASE;  -- END CASE for reset_shaft_encoders 
-- 
--   END CASE;   -- END CASE for reset 
-- 
--   l_cntoutput <= countleft;  -- assign left count   --> passing integer value to another process 
--   r_cntoutput <= countright;  -- assign right count --> passing integer value to another process 
-- 
--END PROCESS count_shaft_feedback; 
-- 
-- this process will sample the output of COUNT_SHAFT_FEEDBACK when appropriate 
-- and then reset the count values in count_shaft_feedback so that new values 
-- can be obtained in the next sampling... 
-- 
-- POSSIBLE PROBLEMS: in the sample_time <= 630, leftcount and rightcount may change as sample_time 
--         goes from 626 to 630.  When should "reset_shaft_counters" be done? 
-- 
-- 
--sample_counters: PROCESS (r_cntoutput, l_cntoutput, clock, reset, reset_shaft_counters) 
-- 
--  variable sample_time: integer; 
-- 
--BEGIN 
-- CASE reset IS 
--   WHEN '1' => 
--  reset_shaft_counters <= '0'; 
--  sample_time := 0; 
--   WHEN OTHERS => 
--  IF (rising_edge(clock)) THEN 
--    IF (sample_time <= 625) THEN 
--      sample_time := sample_time + 1; -- INCREMENT the sample time! 
--      signal_sample_time <= sample_time; 
-- 
--      reset_shaft_counters <= '0'; 
-- 
-- 
--    ELSE 
--     leftcount <= l_cntoutput; 
--     rightcount <= r_cntoutput; 
--     reset_shaft_counters <= '1';  -- hopefully, this will happen concurrently. 
--     sample_time := 0;      -- reset sample_time. 
       --  *************** ACTMAP doesn't like the statement for resetting sample_time to 0. 
       --          because it's being set too many times. 
--    END IF; 
--  END IF; 
  -- IT WON'T DO ANYTHING IF THERE IS NO RISING EDGE. 
-- 
-- END CASE; 
--END PROCESS sample_counters; 
-- *** END of PROCESS count_shaft_feedback ********************************** 
-- **** NEW PROCESS: compare_counters  ******************************* 
-- ******************************************************************* 
-- this process will take the results of rightcount and leftcount and compare them to an optimal number (ref=12) 
-- This number corresponds to the number of shaft encoder pulses desired every 1/4 second, or 625 clock cycles (2500 Hz clock) 
-- 
--compare_counters: PROCESS (rightcount, leftcount, reset, signal_sample_time, halftimel, halftimer,motor_input) 
-- VARIABLE ref: INTEGER; 
-- 
--BEGIN 
-- ref:=12; 
-- CASE reset IS 
--   WHEN '1' => 
--  halftimel <= 25; 
--  halftimer <= 25; 
--   WHEN others => 
--  null; 
-- END CASE; 
-- CASE motor_input IS 
--   WHEN "00" => 
--  IF ((ref > rightcount) AND (signal_sample_time = 625)) THEN 
--    halftimer <= halftimer +1; 
-- 
--  ELSIF ((ref < rightcount) AND (signal_sample_time = 6256)) THEN 
--    halftimer <= halftimer -1; 
-- 
--  ELSE 
--    halftimer <= halftimer;  -- don't change the value of halftimer! 
-- 
--  END IF; 
-- 
--  IF ((ref > leftcount) AND (signal_sample_time = 625)) THEN 
--    halftimel <= halftimel +1; 
-- 
--  ELSIF ((ref < leftcount) AND (signal_sample_time = 625)) THEN 
--    halftimel <= halftimel -1; 
-- 
--  ELSE 
--    halftimel <= halftimel;  -- don't change the value of halftimel! 
-- 
--  END IF; 
-- WHEN others => 
--  null; 
-- END CASE;         -- END the reset CASE 
-- END PROCESS compare_counters; 
-- 
--left_halftime <= halftimel;   -- pass these values on to the next process, pwm. 
--right_halftime <= halftimer; 
-- 
--pwm: PROCESS (clock,reset,left_halftime, right_halftime) 
-- sets up pulse width modulation and adjusts to feedback 
-- 
--VARIABLE time : INTEGER; 
-- 
--BEGIN 
-- 
--   IF (rising_edge(clock)) THEN 
-- 
--  CASE reset IS 
--  WHEN '1' => 
--   time :=0; 
--   leftpulse<='0'; 
--   rightpulse<='0'; 
--  WHEN OTHERS => 
-- 
--  time:=time+1; 
--  IF (time >= 50) THEN 
--   time:=0; 
--  END IF; 
-- 
--  IF (time < left_halftime) THEN --pulse high 
--   leftpulse<='1'; 
--  ELSE 
--   leftpulse<='0'; 
--  END IF; 
-- 
--  IF (time < right_halftime) THEN --pulse high 
--   rightpulse<='1'; 
--  ELSE 
--   rightpulse<='0'; 
--  END IF; 
--  END CASE; 
-- END IF; 
-- 
-- END PROCESS pwm; 
  
-- ******************************************************* 
-- New version of PWM PROCESS **************************** 
-- does NOT use feedback ********************************* 
-- *******************************************************

pwm: PROCESS (clock,reset) 
 -- sets up pulse width modulation. It NO LONGER adjusts to feedback 
 -- This PWM signal only controls the speed of the motors.  Direction is controlled in the LOGIC process. 
 -- We have removed the feedback part of our design due to the ~1000 modules the complete design required.

VARIABLE time : INTEGER; 
VARIABLE halftime : INTEGER; 
   -- In our old version, these two variables were signals from other processes. 
  
BEGIN

   halftime := 1;  -- yields a 50% duty cycle. 
--   right_halftime := 25;  -- yields a 50% duty cycle. 
  
   IF (rising_edge(clock)) THEN

  CASE reset IS 
  WHEN '1' => 
   time :=0; 
   leftpulse<='0'; 
   rightpulse<='0'; 
  WHEN OTHERS => 
  
  time:=time+1; 
  IF (time >= 2) THEN 
   time:=0; 
  END IF;

  IF (time < halftime) THEN -- pulse high 
   leftpulse<='1'; 
  ELSE 
   leftpulse<='0';     -- pulse low 
  END IF;

  IF (time < halftime) THEN -- pulse high 
   rightpulse<='1'; 
  ELSE 
   rightpulse<='0';     -- pulse low 
  END IF; 
  END CASE; 
 END IF; 
  
END PROCESS pwm;

-- ************************************************************************** 
-- counter PROCESS ********************************************************** 
-- ************************************************************************** 
-- 
--counter: PROCESS(input) 
--BEGIN 
-- output <= input +1; 
--END PROCESS counter; 
-- 
end behaviour;