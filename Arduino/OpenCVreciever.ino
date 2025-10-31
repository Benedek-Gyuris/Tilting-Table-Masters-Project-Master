const int maxHistory = 100; // Maximum number of positions to store
int xHistory[maxHistory];   // Array to store X positions
int yHistory[maxHistory];   // Array to store Y positions
unsigned long timeHistory[maxHistory]; // Array to store timestamps
int currentIndex = 0;       // Index to track the current position in the history

String receivedData = "";   // Declare receivedData as a global variable

void setup() {
  Serial.begin(9600); // Initialize serial communication
  Serial.println("Arduino ready to receive positional data and store history.");
}

void loop() {
  // Check if data is available from Python
  if (Serial.available() > 0) {
    char receivedChar = Serial.read(); // Read one character
    if (receivedChar == '\n') { // End of message
      parsePosition(receivedData); // Parse the received data
      receivedData = ""; // Clear the buffer
    } else {
      receivedData += receivedChar; // Append character to string
    }
  }

  // Check if MATLAB is requesting data
  if (Serial.available() > 0) {
    char matlabRequest = Serial.read(); // Read the request character
    if (matlabRequest == 'R') { // 'R' for request
      sendPositionHistory(); // Send the stored position history
    }
  }
}

// Function to parse and store positional data
void parsePosition(String data) {
  int xIndex = data.indexOf('X');
  int yIndex = data.indexOf('Y');

  if (xIndex != -1 && yIndex != -1) {
    // Extract the X and Y values
    int xPos = data.substring(xIndex + 1, yIndex).toInt();
    int yPos = data.substring(yIndex + 1).toInt();

    // Store the position and timestamp in the history
    xHistory[currentIndex] = xPos;
    yHistory[currentIndex] = yPos;
    timeHistory[currentIndex] = millis(); // Store the current time in milliseconds
    currentIndex = (currentIndex + 1) % maxHistory; // Increment and wrap around if needed

    Serial.println("Data updated:");
    Serial.print("X: ");
    Serial.print(xPos);
    Serial.print(", Y: ");
    Serial.println(yPos);
  } else {
    Serial.println("Invalid data format received.");
  }
}

// Function to send the position history to MATLAB
void sendPositionHistory() {
  Serial.println("Sending position history...");
  for (int i = 0; i < currentIndex; i++) {
    Serial.print("Time: ");
    Serial.print(timeHistory[i] / 1000.0); // Convert milliseconds to seconds
    Serial.print("s, X: ");
    Serial.print(xHistory[i]);
    Serial.print(", Y: ");
    Serial.println(yHistory[i]);
  }
  Serial.println("End of history.");
}
