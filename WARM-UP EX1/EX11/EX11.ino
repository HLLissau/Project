#include <LiquidCrystal.h>  // Include the LCD library

// Initialize the LCD with the pin numbers: (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);  

const int switchPin = 6;     // Pin number for the pushbutton
int switchState = 0;         // Current state of the switch
int prevSwitchState = 0;     // Previous state to detect change
int reply;                   // Stores the random answer index

void setup() {
  lcd.begin(16, 2);          // Set up the LCD with 16 columns and 2 rows
  pinMode(switchPin, INPUT); // Set the switch pin as input

  // Display welcome message
  lcd.print("Ask the");
  lcd.setCursor(0, 1);       // Move to the second line
  lcd.print("Crystal Ball!");
}

void loop() {
  switchState = digitalRead(switchPin);  // Read the current switch state

  // Check if the switch was just pressed (change from HIGH to LOW)
  if (switchState != prevSwitchState) {
    if (switchState == LOW) {  // Button is pressed (assuming active-low)

      reply = random(8);       // Generate a random number from 0 to 7

      lcd.clear();             // Clear the LCD screen
      lcd.setCursor(0, 0);     // Go to first line
      lcd.print("The ball says:");
      lcd.setCursor(0, 1);     // Go to second line

      // Choose a message based on the random number
      switch(reply){
        case 0:
          lcd.print("Yes");
          break;
        case 1:
          lcd.print("Most likely");
          break;
        case 2:
          lcd.print("Certainly");
          break;
        case 3:
          lcd.print("Outlook good");
          break;
        case 4:
          lcd.print("Unsure");
          break; 
        case 5:
          lcd.print("Ask again");
          break; 
        case 6:
          lcd.print("Doubtful");
          break; 
        case 7:
          lcd.print("No");
          break;
      }
    }
  }

  // Update previous switch state for next loop
  prevSwitchState = switchState;
}
