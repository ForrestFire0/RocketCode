void setup() {
  Serial.begin(115200);
  Serial.println("Starting Program.");
  //wait for program to send newline.
  readline();
  Serial.println("Hello! We are starting the program.");
  String x;

  while(true) { //Infinitely request more and more data, printing it when recieving.
    long start = millis();
    Serial.println("next");
    x = readline();
    if (x.indexOf("EOF") > -1) {
      break;
    }
    Serial.println("Data recieved");
    start = millis() - start;
    Serial.println(start);
    delay(1000);
  }
  Serial.println("Done with program");
}


void loop() {
  
}

String readline() {
  char serialData;
  String result = "";
  
  while (!Serial.available()); //Wait for serial data to become available.
  
  while (true){ //Loop through each charachter until we reach the newline
    if (Serial.available()) {
      serialData = Serial.read();
      if(serialData == '\n') break;
      result += serialData;
    }
  }

  return result;
}
