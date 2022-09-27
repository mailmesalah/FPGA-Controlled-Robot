/*
** The Program Calculates the Duty Cycle of the given input Signal
** by dividing Input Clock by 2500
**
*/
module Robot(input InputClock, input Reset, input LightSignal,output [1:0]MotorSignal);

  //wire GeneratedClock;
  //wire [7:0] HighCount,LowCount,TotalCount,DutyPercent;
  //wire [3:0] FirstDigit,SecondDigit;

  PulseWidthModilation    M0(InputClock, 2500, GeneratedClock);
  CountHigh               M1(HighCount, GeneratedClock, Signal, Reset);
  CountLow                M2(LowCount, GeneratedClock, Signal, Reset);
  Adder                   M3(HighCount, LowCount, TotalCount);
  CalculateDutyCycle      M4(DutyPercent, HighCount, TotalCount);
  Binary2BCD              M5(DutyPercent, SecondDigit, FirstDigit);
  SevenSegment            M6(FirstDigit, DisplayFirstDigit);
  SevenSegment            M7(SecondDigit, DisplaySecondDigit);

endmodule

module PulseWidthModilation(input InputClock, input [31:0] ClockScale, output reg OutputClock);
/*
** This Module Takes an input InputClock that is the clock given.
** InputClock is counted for 2500 positive edge tick 
** once the count is completed 2500 OutputClock is toggled(ticked) once
**
*/
reg [31:0] ClockCount = 0;

  initial
  begin
    OutputClock=0;
  end
  
  //this module is executed each time InputClock posetive edge is triggered
  always@(posedge InputClock)
  begin
    ClockCount = ClockCount + 1;
    //if the ClockCount reaches the limit(2500) this block is executed
    if (ClockCount >= ClockScale)
    begin
      //toggles the output OutputClock or ticks the OutputClock
      OutputClock = ~OutputClock;
      ClockCount = 0;//resets the count to zero once it reaches 2500
    end
  end

endmodule

module CountHigh(output reg [7:0] HighCount, input Clock, input Signal, input Reset);

  initial
  begin
    HighCount=0;
  end
  
  //The block is executed when ever Clock=1
  always@(posedge Clock)
    
    //if Reset=1 HighCount is set to 0
    if(Reset) 
    begin
      HighCount = 8'b0;
    end
    
    //if Signal=1 and Reset=0 HighCount is incremented
    else if (Signal)
    begin
      HighCount = HighCount + 1;
    end
endmodule

module CountLow(output reg [7:0] LowCount, input Clock, input Signal, input Reset);

  initial
  begin
    LowCount=0;
  end
  
  //The block is executed when ever Clock = 1
  always@(posedge Clock)

    //if Reset=1 LowCount is set to 0
    if(Reset)
    begin
      LowCount = 8'b0;
    end
    
    //if Signal =0 and Reset=0 LowCount is set to 0
    else if (~Signal)
    begin
      LowCount = LowCount + 1;
    end
endmodule

module CalculateDutyCycle(output reg [7:0] DutyCycle, input [7:0] HighCount, input [7:0] TotalCount);

integer x,y;
  initial
  begin
    DutyCycle=0;
  end
  //The block is executed when ever Clock = 1
  always@(HighCount,TotalCount)
    //Calculates the DutyCount 
    
    begin
      
      x=HighCount;
      y=TotalCount;
      y=y-(y%2);
      if (y!=0)
      begin        
        //DutyCycle= (x / y) * 100;
        DutyCycle= (x*100)/y;
      end
    end
endmodule

module Adder(input[7:0] HighCount, input[7:0] LowCount, output reg [7:0] TotalCount);
/*
** This Module is used to add HighCnt with LowCnt and The result is showed through Total
**
*/
  integer i;
  reg [7:0]Store;
  reg [7:0]HighCopy;
  reg [7:0]LowCopy;
  reg AxorB;
  reg AandB;
  reg Sum;
  reg Carry;
  reg AxorB_andC;

  always@(HighCount,LowCount)
    begin
      Carry = 0;
      Sum = 0;
      //Temoporarily stores HighCount in reg HighCopy and LowCount in reg LowCopy
      HighCopy = HighCount;
      LowCopy = LowCount;

      for(i=0; i<=7; i=i+1)
      begin
        //Each bits are added using binary operators
        //HighCopy's i'th bit is XORed with LowCopy's i'th bit
        AxorB = HighCopy[i] ^ LowCopy[i];
        //HighCopy's i'th bit is ANDed with LowcCpy's i'th bit
        AandB = HighCopy[i] & LowCopy[i];
        //AxioB is XORed with Carry of previous iteration. and stored in Sum 
        Sum = AxorB ^ Carry;
        //Calculating the Carry
        //AxorB is ANDed with Carry and stored in AxorB_andC
        AxorB_andC = AxorB & Carry;
        //AandB is Ored with AxorB_andC and stored in Carry for storing current Carry for next iteration
        Carry = AandB | AxorB_andC;
        //sum of i'th iteration is stored in i'th bit of Store
        Store[i] = Sum;
      end
      //Once every bit is added considering their carries, the Sum is stored in TotalCount output register
      TotalCount = Store;
    end
endmodule

module Binary2BCD(input[7:0] Binary, output reg [3:0] SecondDigit, output reg [3:0] FirstDigit);
/*
** This modules Converts the binary data to Binary Coded Decimal format.
** It takes binary value in Binary and outputs the 1s position value of BCD in FirstDigit and 10s position value of BCD in SecondDigit
** Register DataAvailability is used to indicate when the output data is available on output.
*/

reg [7:0] Value;

  initial
  begin
    FirstDigit=0;
    SecondDigit=0;
  end

  //Executes the block when ever clk is positive edge triggered
  always@(posedge Binary)
  begin
    //Binary is copied to Value for working on it
    Value = Binary;
    //First digit is stored in FirstDigit by taking the reminder of Value when divided by 10
    FirstDigit = Value % 10;
    //value is updated with ones digit removed from it
    Value = Value / 10;  
    //Second digit is extracted to SecondDigit taking the reminder by dividing Value with 10
    SecondDigit = Value % 10;
  end
endmodule

module SevenSegment(input [3:0] Digit, output [6:0] Display);
/*
** This module represent the Seven Segment Display 
** Binary Coded Decimal is shown on Seven Segment Display
*/
reg [6:0] TempDisplay;

assign Display = TempDisplay;

  always@(Digit)
  begin
    case(Digit)
      0: TempDisplay = 8'b1111110;
      1: TempDisplay = 8'b0110000;
      2: TempDisplay = 8'b1101101;
      3: TempDisplay = 8'b1111001;
      4: TempDisplay = 8'b0110011;
      5: TempDisplay = 8'b1011011;
      6: TempDisplay = 8'b1011111;
      7: TempDisplay = 8'b1110000;
      8: TempDisplay = 8'b1111111;
      9: TempDisplay = 8'b1111011;
    endcase
  end
endmodule
