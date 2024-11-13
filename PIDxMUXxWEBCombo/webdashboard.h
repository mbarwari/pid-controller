#ifndef WEBDASHBOARD_H
#define WEBDASHBOARD_H
#define ResetPin 30  // Change 5 to any available digital pin

#include <WiFi.h>
#include "chartjs.h"  // Include the chart.js content

// Error message declaration
String currentErrorMessage = ""; // Initialize to an empty string

// Function prototypes
void setupWebDashboard(WiFiServer& server);
void handleWebRequests(WiFiServer& server);

//Adjustable variables 
extern double Kp_peltier, Ki_peltier, Kd_peltier;
extern double SetpointPeltier, SetpointWB;

//column 1
//Define thermistor references 
extern float steinhartIntraArray1;
extern float steinhartIntraArray2; 
extern float steinhartIntraArray3;
extern float steinhartIntraArray4;
extern float steinhartExtraArray1;
extern float steinhartExtraArray2;
extern float steinhartExtraArray3;
extern float steinhartSWB1;
extern float steinhartSWB2;
extern float steinhartSWB3;
extern float steinhartEntrBWB1;
extern float steinhartEntrBWB2;
extern float steinhartEntrBWB3;
extern float steinhartBWB1;
extern float steinhartBWB2;
extern float steinhartBWB3;
extern float steinhartBWB4;
extern float steinhartBWB5;
extern float steinhartBWB6;
extern float steinhartExitBWB1;
extern float steinhartExitBWB2;
extern float steinhartExitBWB3;

//column 2
//Define avg thermistor references
extern float braintemp_atm; 
extern float ExtraArrayTemp;
extern float SWBTemp_atm;
extern float EntrBWBTemp;
extern float BWBTemp;
extern float ExitBWBTemp;

//column 3
//Define flow, pressure, current, and voltage references 
extern float pressureApplied1;
extern float pressureApplied2;
extern float aFlow;
extern float bFlow;
extern float aTemperature;
extern float bTemperature;
extern float pumpVoltage;
extern float pumpCurrent;
extern float peltierVoltage;
extern float peltierCurrent;
extern float previousFreq; 
extern float dacVoltage; 

// Web dashboard error handling
void updateErrorStatus(String errorMessage) {
    // Use a global variable to store the latest error message
    currentErrorMessage = errorMessage;
    // Update the webpage when new error messages are available
}

// Function to set up the web dashboard (called in setup)
void setupWebDashboard(WiFiServer& server) {
  // Start the server
  server.begin();
  Serial.println("WebDashboard server started.");
  Serial.print("Access Point IP Address: ");
  Serial.println(WiFi.localIP());
}

// Function to handle incoming web requests (called in loop)
void handleWebRequests(WiFiServer& server) {
  WiFiClient client = server.available();
  if (client && client.connected()) {
    String request = client.readStringUntil('\r');
    client.flush();

    //Reseting Arduino
    // New route to handle digital signal trigger
    if (request.indexOf("GET /triggerDigitalSignal") >= 0) {
      // Set the digital pin HIGH for a short pulse
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.print("Signal Sent");
      client.stop();
      
      digitalWrite(ResetPin, LOW);
      
      return;
    }
    
    // Serve the chart.js content when requested
    if (request.indexOf("GET /chartjs.js") >= 0) {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: application/javascript");
      client.println();
      client.write(chart_js);  // Serve chart.js library
      client.stop();
      return;
    }

    // Serve the current values when requested
    if (request.indexOf("GET /getValues") >= 0) {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.print(Kp_peltier);
      client.print(",");
      client.print(Ki_peltier);
      client.print(",");
      client.print(Kd_peltier);
      client.print(",");
      client.print(SetpointPeltier);
      client.print(",");
      client.print(SetpointWB);
      client.print(",");
      client.print(SWBTemp_atm);
      client.print(",");
      client.print(braintemp_atm);
      client.print(",");
      client.print(aFlow);
      client.print(",");
      client.print(bFlow);
      client.print(",");
      client.print(pumpVoltage);
      client.print(",");
      client.print(pumpCurrent);
      client.print(",");
      client.print(peltierVoltage);
      client.print(",");
      client.print(peltierCurrent);
      client.print(",");
      client.print(pressureApplied1);
      client.print(",");
      client.print(pressureApplied2);
      client.print(",");
      client.print(previousFreq);
      client.print(",");
      client.print(dacVoltage);
      client.stop();
      return;
    }

/*
        // Serve the current values when requested
    if (request.indexOf("GET /getValues") >= 0) {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/plain");
        client.println();
        client.print(Kp_peltier);            
        client.print(",");
        client.print(Ki_peltier);            
        client.print(",");
        client.print(Kd_peltier);            
        client.print(",");
        client.print(SetpointPeltier);      
        client.print(",");
        client.print(SetpointWB);           
        client.print(",");
        client.print(steinhartIntraArray1);  
        client.print(",");
        client.print(steinhartIntraArray2);  
        client.print(",");
        client.print(steinhartIntraArray3);  
        client.print(",");
        client.print(steinhartIntraArray4);  
        client.print(",");
        client.print(steinhartExtraArray1);  
        client.print(",");
        client.print(steinhartExtraArray2);  
        client.print(",");
        client.print(steinhartExtraArray3);  
        client.print(",");
        client.print(steinhartSWB1);         
        client.print(",");
        client.print(steinhartSWB2);         
        client.print(",");
        client.print(steinhartSWB3);         
        client.print(",");
        client.print(steinhartEntrBWB1);     
        client.print(",");
        client.print(steinhartEntrBWB2);     
        client.print(",");
        client.print(steinhartEntrBWB3);     
        client.print(",");
        client.print(steinhartBWB1);         
        client.print(",");
        client.print(steinhartBWB2);         
        client.print(",");
        client.print(steinhartBWB3);         
        client.print(",");
        client.print(steinhartBWB4);         
        client.print(",");
        client.print(steinhartBWB5);         
        client.print(",");
        client.print(steinhartBWB6);         
        client.print(",");
        client.print(steinhartExitBWB1);     
        client.print(",");
        client.print(steinhartExitBWB2);     
        client.print(",");
        client.print(steinhartExitBWB3);     
        client.print(",");
        client.print(braintemp_atm);         
        client.print(",");
        client.print(ExtraArrayTemp);        
        client.print(",");
        client.print(SWBTemp_atm);           
        client.print(",");
        client.print(EntrBWBTemp);           
        client.print(",");
        client.print(BWBTemp);               
        client.print(",");
        client.print(ExitBWBTemp);           
        client.print(",");
        client.print(pressureApplied1);     
        client.print(",");
        client.print(pressureApplied2);      
        client.print(",");
        client.print(aFlow);                
        client.print(",");
        client.print(bFlow);                
        client.print(",");
        client.print(aTemperature);         
        client.print(",");
        client.print(bTemperature);         
        client.print(",");
        client.print(pumpVoltage);          
        client.print(",");
        client.print(pumpCurrent);          
        client.print(",");
        client.print(peltierVoltage);       
        client.print(",");
        client.print(peltierCurrent);       
        client.print(",");
        client.print(pressureApplied1);     
        client.print(",");
        client.print(pressureApplied2);     
        client.print(",");
        client.print(previousFreq);         
        client.print(",");
        client.print(dacVoltage);           
        client.print(",");
        // Close the connection
        client.stop();
        return;
    }
*/

    // Update Kp_peltier, Ki_peltier, Kd_peltier, SetpointPeltier, and SetpointWB via HTTP requests
    if (request.indexOf("GET /updateKp_peltier?newVal=") >= 0) {
      String newVal = request.substring(request.indexOf("newVal=") + 7);
      Kp_peltier = newVal.toDouble();
      Serial.print("Kp_peltier updated to: ");
      Serial.println(Kp_peltier);
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("OK");
      client.stop();
      return;
    }
    if (request.indexOf("GET /updateKi_peltier?newVal=") >= 0) {
      String newVal = request.substring(request.indexOf("newVal=") + 7);
      Ki_peltier = newVal.toDouble();
      Serial.print("Ki_peltier updated to: ");
      Serial.println(Ki_peltier);
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("OK");
      client.stop();
      return;
    }
    if (request.indexOf("GET /updateKd_peltier?newVal=") >= 0) {
      String newVal = request.substring(request.indexOf("newVal=") + 7);
      Kd_peltier = newVal.toDouble();
      Serial.print("Kd_peltier updated to: ");
      Serial.println(Kd_peltier);
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("OK");
      client.stop();
      return;
    }
    if (request.indexOf("GET /updateSetpointPeltier?newVal=") >= 0) {
      String newVal = request.substring(request.indexOf("newVal=") + 7);
      SetpointPeltier = newVal.toDouble();
      Serial.print("SetpointPeltier updated to: ");
      Serial.println(SetpointPeltier);
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("OK");
      client.stop();
      return;
    }
    if (request.indexOf("GET /updateSetpointWB?newVal=") >= 0) {
      String newVal = request.substring(request.indexOf("newVal=") + 7);
      SetpointWB = newVal.toDouble();
      Serial.print("SetpointWB updated to: ");
      Serial.println(SetpointWB);
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println();
      client.println("OK");
      client.stop();
      return;
    }

    // Serve the HTML page with the form and the graphs
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<!DOCTYPE HTML><html><head><title>Giga R1 Web Dashboard</title>");
    client.println("<script src=\"/chartjs.js\"></script>");  
    client.println("</head><body>");
    client.println("<h1>Live Data Display</h1>");

    // Forms for updating Kp_peltier, Ki_peltier, Kd_peltier, SetpointPeltier, and SetpointWB
    client.println("<form id='updateFormKp_peltier' onsubmit='submitForm(event, \"Kp_peltier\")'>");
    client.println("<label for=\"newValKp_peltier\">Update Kp_peltier (Proportional):</label><br>");
    client.println("<input type=\"number\" step=\"0.1\" id=\"newValKp_peltier\" name=\"newVal\"><br>");
    client.println("<button type='submit'>Submit Kp_peltier</button>");
    client.println("</form><br>");

    client.println("<form id='updateFormKi_peltier' onsubmit='submitForm(event, \"Ki_peltier\")'>");
    client.println("<label for=\"newValKi_peltier\">Update Ki_peltier (Integral):</label><br>");
    client.println("<input type=\"number\" step=\"0.1\" id=\"newValKi_peltier\" name=\"newVal\"><br>");
    client.println("<button type='submit'>Submit Ki_peltier</button>");
    client.println("</form><br>");

    client.println("<form id='updateFormKd_peltier' onsubmit='submitForm(event, \"Kd_peltier\")'>");
    client.println("<label for=\"newValKd_peltier\">Update Kd_peltier (Derivative):</label><br>");
    client.println("<input type=\"number\" step=\"0.1\" id=\"newValKd_peltier\" name=\"newVal\"><br>");
    client.println("<button type='submit'>Submit Kd_peltier</button>");
    client.println("</form><br>");

    client.println("<form id='updateFormSetpointPeltier' onsubmit='submitForm(event, \"SetpointPeltier\")'>");
    client.println("<label for=\"newValSetpointPeltier\">Update Setpoint Peltier:</label><br>");
    client.println("<input type=\"number\" step=\"0.1\" id=\"newValSetpointPeltier\" name=\"newVal\"><br>");
    client.println("<button type='submit'>Submit Setpoint</button>");
    client.println("</form><br>");

    client.println("<form id='updateFormSetpointWB' onsubmit='submitForm(event, \"SetpointWB\")'>");
    client.println("<label for=\"newValSetpointWB\">Update Setpoint WB:</label><br>");
    client.println("<input type=\"number\" step=\"0.1\" id=\"newValSetpointWB\" name=\"newVal\"><br>");
    client.println("<button type='submit'>Submit Setpoint</button>");
    client.println("</form><br>");


    // tables 
    client.println("<style>");
    client.println("  .table-container {");
    client.println("    display: flex;");
    client.println("    justify-content: space-around;");
    client.println("  }");
    client.println("  .table {");
    client.println("    margin: 10px;");
    client.println("    border-collapse: collapse;");
    client.println("  }");
    client.println("  .table th, .table td {");
    client.println("    padding: 8px 12px;");
    client.println("    border: 1px solid #ddd;");
    client.println("    border: 1px solid #ddd;");
    client.println("    height: 50px;"); // Set a height for the cells to accommodate the button
    client.println("  }");
    client.println("  .actionButton {");
    client.println("    width: 100%;");
    client.println("    height: 100%;");
    client.println("    font-size: 18px;");
    client.println("    border: none;");
    client.println("    color: white;");
    client.println("    text-align: center;");
    client.println("    cursor: pointer;");
    client.println("  }");
    client.println("</style>");

    client.println("<div class='table-container'>");

    client.println("<div class='table'>");
    client.println("<table>");
    client.println("<tr><th>Parameter</th><th>Value</th><th>Action</th></tr>");
    client.println("<tr><td>Proportional</td><td><span id='proportionalValue'>0</span></td><td><button id='btnProportional' class='actionButton' onclick='toggleButtonColor(\"btnProportional\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Integral</td><td><span id='integralValue'>0</span></td><td><button id='btnIntegral' class='actionButton' onclick='toggleButtonColor(\"btnIntegral\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Derivative</td><td><span id='derivativeValue'>0</span></td><td><button id='btnDerivative' class='actionButton' onclick='toggleButtonColor(\"btnDerivative\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Setpoint Peltier</td><td><span id='setpointPeltierValue'>" + String(SetpointPeltier) + "</span></td><td><button id='btnSetpointPeltier' class='actionButton' onclick='toggleButtonColor(\"btnSetpointPeltier\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Setpoint WB</td><td><span id='setpointWBValue'>" + String(SetpointWB) + "</span></td><td><button id='btnSetpointWB' class='actionButton' onclick='toggleButtonColor(\"btnSetpointWB\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Scalp Water Block Temperature</td><td><span id='SWBTempValue'>0.0</span></td><td><button id='btnSWBTemp' class='actionButton' onclick='toggleButtonColor(\"btnSWBTemp\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Brain Temperature</td><td><span id='brainTempValue'>0.0</span></td><td><button id='btnBrainTemp' class='actionButton' onclick='toggleButtonColor(\"btnBrainTemp\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Flow Rate A</td><td><span id='aFlowValue'>0.0</span></td><td><button id='btnFlowA' class='actionButton' onclick='toggleButtonColor(\"btnFlowA\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Flow Rate B</td><td><span id='bFlowValue'>0.0</span></td><td><button id='btnFlowB' class='actionButton' onclick='toggleButtonColor(\"btnFlowB\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pump Voltage</td><td><span id='pumpVoltageValue'>0.0</span></td><td><button id='btnPumpVoltage' class='actionButton' onclick='toggleButtonColor(\"btnPumpVoltage\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pump Current</td><td><span id='pumpCurrentValue'>0.0</span></td><td><button id='btnPumpCurrent' class='actionButton' onclick='toggleButtonColor(\"btnPumpCurrent\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Peltier Voltage</td><td><span id='peltierVoltageValue'>0.0</span></td><td><button id='btnPeltierVoltage' class='actionButton' onclick='toggleButtonColor(\"btnPeltierVoltage\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Peltier Current</td><td><span id='peltierCurrentValue'>0.0</span></td><td><button id='btnPeltierCurrent' class='actionButton' onclick='toggleButtonColor(\"btnPeltierCurrent\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pressure Applied 1</td><td><span id='pressure1Value'>0.0</span></td><td><button id='btnPressure1' class='actionButton' onclick='toggleButtonColor(\"btnPressure1\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pressure Applied 2</td><td><span id='pressure2Value'>0.0</span></td><td><button id='btnPressure2' class='actionButton' onclick='toggleButtonColor(\"btnPressure2\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Frequency Output</td><td><span id='previousFreqValue'>0.0</span></td><td><button id='btnFreqOutput' class='actionButton' onclick='toggleButtonColor(\"btnFreqOutput\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>DAC Voltage</td><td><span id='dacVoltageValue'>0.0</span></td><td><button id='btnDACVoltage' class='actionButton' onclick='toggleButtonColor(\"btnDACVoltage\")' style='background-color: green;'>1</button></td></tr>");
    client.println("</table>");
    client.println("</div>");

    client.println("<div class='table'>");
    client.println("<table>");
    client.println("<tr><th>Parameter</th><th>Value</th><th>Action</th></tr>");
    client.println("<tr><td>Proportional</td><td><span id='proportionalValue'>0</span></td><td><button id='btnProportional' class='actionButton' onclick='toggleButtonColor(\"btnProportional\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Integral</td><td><span id='integralValue'>0</span></td><td><button id='btnIntegral' class='actionButton' onclick='toggleButtonColor(\"btnIntegral\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Derivative</td><td><span id='derivativeValue'>0</span></td><td><button id='btnDerivative' class='actionButton' onclick='toggleButtonColor(\"btnDerivative\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Setpoint Peltier</td><td><span id='setpointPeltierValue'>" + String(SetpointPeltier) + "</span></td><td><button id='btnSetpointPeltier' class='actionButton' onclick='toggleButtonColor(\"btnSetpointPeltier\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Setpoint WB</td><td><span id='setpointWBValue'>" + String(SetpointWB) + "</span></td><td><button id='btnSetpointWB' class='actionButton' onclick='toggleButtonColor(\"btnSetpointWB\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Scalp Water Block Temperature</td><td><span id='SWBTempValue'>0.0</span></td><td><button id='btnSWBTemp' class='actionButton' onclick='toggleButtonColor(\"btnSWBTemp\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Brain Temperature</td><td><span id='brainTempValue'>0.0</span></td><td><button id='btnBrainTemp' class='actionButton' onclick='toggleButtonColor(\"btnBrainTemp\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Flow Rate A</td><td><span id='aFlowValue'>0.0</span></td><td><button id='btnFlowA' class='actionButton' onclick='toggleButtonColor(\"btnFlowA\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Flow Rate B</td><td><span id='bFlowValue'>0.0</span></td><td><button id='btnFlowB' class='actionButton' onclick='toggleButtonColor(\"btnFlowB\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pump Voltage</td><td><span id='pumpVoltageValue'>0.0</span></td><td><button id='btnPumpVoltage' class='actionButton' onclick='toggleButtonColor(\"btnPumpVoltage\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pump Current</td><td><span id='pumpCurrentValue'>0.0</span></td><td><button id='btnPumpCurrent' class='actionButton' onclick='toggleButtonColor(\"btnPumpCurrent\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Peltier Voltage</td><td><span id='peltierVoltageValue'>0.0</span></td><td><button id='btnPeltierVoltage' class='actionButton' onclick='toggleButtonColor(\"btnPeltierVoltage\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Peltier Current</td><td><span id='peltierCurrentValue'>0.0</span></td><td><button id='btnPeltierCurrent' class='actionButton' onclick='toggleButtonColor(\"btnPeltierCurrent\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pressure Applied 1</td><td><span id='pressure1Value'>0.0</span></td><td><button id='btnPressure1' class='actionButton' onclick='toggleButtonColor(\"btnPressure1\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pressure Applied 2</td><td><span id='pressure2Value'>0.0</span></td><td><button id='btnPressure2' class='actionButton' onclick='toggleButtonColor(\"btnPressure2\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Frequency Output</td><td><span id='previousFreqValue'>0.0</span></td><td><button id='btnFreqOutput' class='actionButton' onclick='toggleButtonColor(\"btnFreqOutput\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>DAC Voltage</td><td><span id='dacVoltageValue'>0.0</span></td><td><button id='btnDACVoltage' class='actionButton' onclick='toggleButtonColor(\"btnDACVoltage\")' style='background-color: green;'>1</button></td></tr>");client.println("</table>");
    client.println("</div>");

    client.println("<div class='table'>");
    client.println("<table>");
    client.println("<tr><th>Parameter</th><th>Value</th><th>Action</th></tr>");
    client.println("<tr><td>Proportional</td><td><span id='proportionalValue'>0</span></td><td><button id='btnProportional' class='actionButton' onclick='toggleButtonColor(\"btnProportional\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Integral</td><td><span id='integralValue'>0</span></td><td><button id='btnIntegral' class='actionButton' onclick='toggleButtonColor(\"btnIntegral\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Derivative</td><td><span id='derivativeValue'>0</span></td><td><button id='btnDerivative' class='actionButton' onclick='toggleButtonColor(\"btnDerivative\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Setpoint Peltier</td><td><span id='setpointPeltierValue'>" + String(SetpointPeltier) + "</span></td><td><button id='btnSetpointPeltier' class='actionButton' onclick='toggleButtonColor(\"btnSetpointPeltier\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Setpoint WB</td><td><span id='setpointWBValue'>" + String(SetpointWB) + "</span></td><td><button id='btnSetpointWB' class='actionButton' onclick='toggleButtonColor(\"btnSetpointWB\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Scalp Water Block Temperature</td><td><span id='SWBTempValue'>0.0</span></td><td><button id='btnSWBTemp' class='actionButton' onclick='toggleButtonColor(\"btnSWBTemp\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Brain Temperature</td><td><span id='brainTempValue'>0.0</span></td><td><button id='btnBrainTemp' class='actionButton' onclick='toggleButtonColor(\"btnBrainTemp\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Flow Rate A</td><td><span id='aFlowValue'>0.0</span></td><td><button id='btnFlowA' class='actionButton' onclick='toggleButtonColor(\"btnFlowA\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Flow Rate B</td><td><span id='bFlowValue'>0.0</span></td><td><button id='btnFlowB' class='actionButton' onclick='toggleButtonColor(\"btnFlowB\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pump Voltage</td><td><span id='pumpVoltageValue'>0.0</span></td><td><button id='btnPumpVoltage' class='actionButton' onclick='toggleButtonColor(\"btnPumpVoltage\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pump Current</td><td><span id='pumpCurrentValue'>0.0</span></td><td><button id='btnPumpCurrent' class='actionButton' onclick='toggleButtonColor(\"btnPumpCurrent\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Peltier Voltage</td><td><span id='peltierVoltageValue'>0.0</span></td><td><button id='btnPeltierVoltage' class='actionButton' onclick='toggleButtonColor(\"btnPeltierVoltage\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Peltier Current</td><td><span id='peltierCurrentValue'>0.0</span></td><td><button id='btnPeltierCurrent' class='actionButton' onclick='toggleButtonColor(\"btnPeltierCurrent\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pressure Applied 1</td><td><span id='pressure1Value'>0.0</span></td><td><button id='btnPressure1' class='actionButton' onclick='toggleButtonColor(\"btnPressure1\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Pressure Applied 2</td><td><span id='pressure2Value'>0.0</span></td><td><button id='btnPressure2' class='actionButton' onclick='toggleButtonColor(\"btnPressure2\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>Frequency Output</td><td><span id='previousFreqValue'>0.0</span></td><td><button id='btnFreqOutput' class='actionButton' onclick='toggleButtonColor(\"btnFreqOutput\")' style='background-color: green;'>1</button></td></tr>");
    client.println("<tr><td>DAC Voltage</td><td><span id='dacVoltageValue'>0.0</span></td><td><button id='btnDACVoltage' class='actionButton' onclick='toggleButtonColor(\"btnDACVoltage\")' style='background-color: green;'>1</button></td></tr>");client.println("</table>");
    client.println("</div>");

    client.println("</div>");  // Close the table-container div

    client.println("<script>");
    client.println("window.onload = function() {");
    client.println("  // Restore the button states from localStorage when the page loads");
    client.println("  var buttonIds = [\"btnProportional\", \"btnIntegral\", \"btnDerivative\", \"btnSetpointPeltier\", \"btnSetpointWB\", \"btnSWBTemp\", \"btnBrainTemp\", \"btnFlowA\", \"btnFlowB\", \"btnPumpVoltage\", \"btnPumpCurrent\", \"btnPeltierVoltage\", \"btnPeltierCurrent\", \"btnPressure1\", \"btnPressure2\", \"btnFreqOutput\", \"btnDACVoltage\"];");
    client.println("  for (var i = 0; i < buttonIds.length; i++) {");
    client.println("    var btn = document.getElementById(buttonIds[i]);");
    client.println("    var state = localStorage.getItem(buttonIds[i]);");
    client.println("    if (state === '0') {");
    client.println("      btn.style.backgroundColor = 'red';");
    client.println("      btn.innerHTML = '0';");
    client.println("    } else {");
    client.println("      btn.style.backgroundColor = 'green';");
    client.println("      btn.innerHTML = '1';");
    client.println("    }");
    client.println("  }");
    client.println("};");

    client.println("function toggleButtonColor(buttonId) {");
    client.println("  var btn = document.getElementById(buttonId);");
    client.println("  if (btn.style.backgroundColor === 'red') {");
    client.println("    btn.style.backgroundColor = 'green';");
    client.println("    btn.innerHTML = '1';");
    client.println("    localStorage.setItem(buttonId, '1');");
    client.println("  } else {");
    client.println("    btn.style.backgroundColor = 'red';");
    client.println("    btn.innerHTML = '0';");
    client.println("    localStorage.setItem(buttonId, '0');");
    client.println("  }");
    client.println("}");
    client.println("</script>");




//

    // Canvas for charts
    client.println("<canvas id='proportionalChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='temperatureChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='flowRateChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='powerChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='pressureChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='frequencyChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='dacChart' style='width: 200px; height: 100px;'></canvas>");

    // Button to download CSV
    client.println("<button onclick='downloadCSV()'>Download CSV</button>");

    //Button to reset Arduino
    client.println("<button onclick='sendSignal()'>Reset Arduino</button>");


    // JavaScript to update the value, the chart, and download data as CSV
    client.println("<script>");
    client.println("let dataProportional = [];");
    client.println("let dataIntegral = [];");
    client.println("let dataDerivative = [];");
    client.println("let dataSetpointPeltier = [];");
    client.println("let dataSetpointWB = [];");
    client.println("let dataSWBTemp = [];");
    client.println("let dataBrainTemp = [];");
    client.println("let dataAFlow = [];");
    client.println("let dataBFlow = [];");
    client.println("let dataPumpVoltage = [];");
    client.println("let dataPumpCurrent = [];");
    client.println("let dataPeltierVoltage = [];");
    client.println("let dataPeltierCurrent = [];");
    client.println("let dataPressure1 = [];");
    client.println("let dataPressure2 = [];");
    client.println("let dataPreviousFreq = [];");
    client.println("let dataDacVoltage = [];");
    client.println("let labels = [];");
    client.println("let time = 0;");
    /*

    client.println("<script>");
    client.println("let dataKpPeltier = [];");   // Adjustable variable
    client.println("let dataKiPeltier = [];");   // Adjustable variable
    client.println("let dataKdPeltier = [];");   // Adjustable variable
    client.println("let dataSetpointPeltier = [];");  // Adjustable variable
    client.println("let dataSetpointWB = [];");      // Adjustable variable
    client.println("let dataSteinhartIntraArray1 = [];"); 
    client.println("let dataSteinhartIntraArray2 = [];"); 
    client.println("let dataSteinhartIntraArray3 = [];"); 
    client.println("let dataSteinhartIntraArray4 = [];"); 
    client.println("let dataSteinhartExtraArray1 = [];"); 
    client.println("let dataSteinhartExtraArray2 = [];"); 
    client.println("let dataSteinhartExtraArray3 = [];"); 
    client.println("let dataSteinhartSWB1 = [];"); 
    client.println("let dataSteinhartSWB2 = [];"); 
    client.println("let dataSteinhartSWB3 = [];"); 
    client.println("let dataSteinhartEntrBWB1 = [];"); 
    client.println("let dataSteinhartEntrBWB2 = [];"); 
    client.println("let dataSteinhartEntrBWB3 = [];"); 
    client.println("let dataSteinhartBWB1 = [];"); 
    client.println("let dataSteinhartBWB2 = [];"); 
    client.println("let dataSteinhartBWB3 = [];"); 
    client.println("let dataSteinhartBWB4 = [];"); 
    client.println("let dataSteinhartBWB5 = [];"); 
    client.println("let dataSteinhartBWB6 = [];"); 
    client.println("let dataSteinhartExitBWB1 = [];"); 
    client.println("let dataSteinhartExitBWB2 = [];"); 
    client.println("let dataSteinhartExitBWB3 = [];"); 
    client.println("let dataBrainTempAtm = [];"); 
    client.println("let dataExtraArrayTemp = [];"); 
    client.println("let dataSWBTempAtm = [];"); 
    client.println("let dataEntrBWBTemp = [];"); 
    client.println("let dataBWBTemp = [];"); 
    client.println("let dataExitBWBTemp = [];"); 
    client.println("let dataPressureApplied1 = [];"); 
    client.println("let dataPressureApplied2 = [];"); 
    client.println("let dataAFlow = [];"); 
    client.println("let dataBFlow = [];"); 
    client.println("let dataATemperature = [];"); 
    client.println("let dataBTemperature = [];"); 
    client.println("let dataPumpVoltage = [];"); 
    client.println("let dataPumpCurrent = [];"); 
    client.println("let dataPeltierVoltage = [];"); 
    client.println("let dataPeltierCurrent = [];"); 
    client.println("let dataPreviousFreq = [];"); 
    client.println("let dataDacVoltage = [];");
    client.println("let labels = [];");  // Add time-based labels
    client.println("let time = 0;");  // Counter to simulate time progression
*/

    // Initialize the charts
    // Initialize PID chart
    client.println("let ctx1 = document.getElementById('proportionalChart').getContext('2d');");
    client.println("let chart1 = new Chart(ctx1, {");
    client.println("    type: 'line',");
    client.println("    data: {");
    client.println("      labels: labels,");
    client.println("      datasets: [{");
    client.println("        label: 'Proportional',");
    client.println("        data: dataProportional,");
    client.println("        borderColor: 'blue',");
    client.println("        fill: false");
    client.println("      }, {");
    client.println("        label: 'Integral',");
    client.println("        data: dataIntegral,");
    client.println("        borderColor: 'green',");
    client.println("        fill: false");
    client.println("      }, {");
    client.println("        label: 'Derivative',");
    client.println("        data: dataDerivative,");
    client.println("        borderColor: 'red',");
    client.println("        fill: false");
    client.println("      }]");
    client.println("    },");
    client.println("    options: {");
    client.println("      scales: {");
    client.println("        x: { title: { display: true, text: 'Time' } },");
    client.println("        y: { title: { display: true, text: 'Value' } }");
    client.println("      }");
    client.println("    }");
    client.println("});");

    // Initialize the temperature chart
    client.println("let ctx2 = document.getElementById('temperatureChart').getContext('2d');");
    client.println("let tempChart = new Chart(ctx2, {");
    client.println("    type: 'line',");
    client.println("    data: {");
    client.println("      labels: labels,");
    client.println("      datasets: [{");
    client.println("        label: 'Scalp Water Block Temp',");
    client.println("        data: dataSWBTemp,");
    client.println("        borderColor: 'orange',");
    client.println("        fill: false");
    client.println("      }, {");
    client.println("        label: 'Brain Temp',");
    client.println("        data: dataBrainTemp,");
    client.println("        borderColor: 'purple',");
    client.println("        fill: false");
    client.println("      }]");
    client.println("    },");
    client.println("    options: {");
    client.println("      scales: {");
    client.println("        x: { title: { display: true, text: 'Time' } },");
    client.println("        y: { title: { display: true, text: 'Temperature (C)' } }");
    client.println("      }");
    client.println("    }");
    client.println("});");

    // Initialize the flow rate chart
    client.println("let ctx3 = document.getElementById('flowRateChart').getContext('2d');");
    client.println("let flowRateChart = new Chart(ctx3, {");
    client.println("    type: 'line',");
    client.println("    data: {");
    client.println("      labels: labels,");
    client.println("      datasets: [{");
    client.println("        label: 'Flow Rate A',");
    client.println("        data: dataAFlow,");
    client.println("        borderColor: 'cyan',");
    client.println("        fill: false");
    client.println("      }, {");
    client.println("        label: 'Flow Rate B',");
    client.println("        data: dataBFlow,");
    client.println("        borderColor: 'magenta',");
    client.println("        fill: false");
    client.println("      }]");
    client.println("    },");
    client.println("    options: {");
    client.println("      scales: {");
    client.println("        x: { title: { display: true, text: 'Time' } },");
    client.println("        y: { title: { display: true, text: 'Flow Rate (L/min)' } }");
    client.println("      }");
    client.println("    }");
    client.println("});");

    // Initialize the power chart for pump and peltier data
    client.println("let ctx4 = document.getElementById('powerChart').getContext('2d');");
    client.println("let powerChart = new Chart(ctx4, {");
    client.println("    type: 'line',");
    client.println("    data: {");
    client.println("      labels: labels,");
    client.println("      datasets: [{");
    client.println("        label: 'Pump Voltage',");
    client.println("        data: dataPumpVoltage,");
    client.println("        borderColor: 'blue',");
    client.println("        fill: false");
    client.println("      }, {");
    client.println("        label: 'Pump Current',");
    client.println("        data: dataPumpCurrent,");
    client.println("        borderColor: 'green',");
    client.println("        fill: false");
    client.println("      }, {");
    client.println("        label: 'Peltier Voltage',");
    client.println("        data: dataPeltierVoltage,");
    client.println("        borderColor: 'orange',");
    client.println("        fill: false");
    client.println("      }, {");
    client.println("        label: 'Peltier Current',");
    client.println("        data: dataPeltierCurrent,");
    client.println("        borderColor: 'red',");
    client.println("        fill: false");
    client.println("      }]");
    client.println("    },");
    client.println("    options: {");
    client.println("      scales: {");
    client.println("        x: { title: { display: true, text: 'Time' } },");
    client.println("        y: { title: { display: true, text: 'Voltage / Current (V/A)' } }");
    client.println("      }");
    client.println("    }");
    client.println("});");

    // Initialize the pressure chart
    client.println("let ctx5 = document.getElementById('pressureChart').getContext('2d');");
    client.println("let pressureChart = new Chart(ctx5, {");
    client.println("    type: 'line',");
    client.println("    data: {");
    client.println("      labels: labels,");
    client.println("      datasets: [{");
    client.println("        label: 'Pressure Applied 1',");
    client.println("        data: dataPressure1,");
    client.println("        borderColor: 'purple',");
    client.println("        fill: false");
    client.println("      }, {");
    client.println("        label: 'Pressure Applied 2',");
    client.println("        data: dataPressure2,");
    client.println("        borderColor: 'brown',");
    client.println("        fill: false");
    client.println("      }]");
    client.println("    },");
    client.println("    options: {");
    client.println("      scales: {");
    client.println("        x: { title: { display: true, text: 'Time' } },");
    client.println("        y: { title: { display: true, text: 'Pressure (PSI)' } }");
    client.println("      }");
    client.println("    }");
    client.println("});");

    // Initialize the frequency chart
    client.println("let ctx6 = document.getElementById('frequencyChart').getContext('2d');");
    client.println("let frequencyChart = new Chart(ctx6, {");
    client.println("    type: 'line',");
    client.println("    data: {");
    client.println("      labels: labels,");
    client.println("      datasets: [{");
    client.println("        label: 'Frequency Output',");
    client.println("        data: dataPreviousFreq,");
    client.println("        borderColor: 'blue',");
    client.println("        fill: false");
    client.println("      }]");
    client.println("    },");
    client.println("    options: {");
    client.println("      scales: {");
    client.println("        x: { title: { display: true, text: 'Time' } },");
    client.println("        y: { title: { display: true, text: 'Frequency (Hz)' } }");
    client.println("      }");
    client.println("    }");
    client.println("});");

    // Initialize the frequency chart
    client.println("let ctx7 = document.getElementById('dacChart').getContext('2d');");
    client.println("let dacChart = new Chart(ctx7, {");
    client.println("    type: 'line',");
    client.println("    data: {");
    client.println("      labels: labels,");
    client.println("      datasets: [{");
    client.println("        label: 'DAC Output',");
    client.println("        data: dataDacVoltage,");
    client.println("        borderColor: 'red',");
    client.println("        fill: false");
    client.println("      }]");
    client.println("    },");
    client.println("    options: {");
    client.println("      scales: {");
    client.println("        x: { title: { display: true, text: 'Time' } },");
    client.println("        y: { title: { display: true, text: 'Voltage (V)' } }");
    client.println("      }");
    client.println("    }");
    client.println("});");


    // Function to update the chart and value without refreshing the page
    client.println("function updateValue() {");
    client.println("  let xhr = new XMLHttpRequest();");
    client.println("  xhr.onreadystatechange = function() {");
    client.println("    if (xhr.readyState == 4 && xhr.status == 200) {");
    client.println("      let values = xhr.responseText.split(',');");
    client.println("      let Kp_peltier = parseFloat(values[0]);");
    client.println("      let Ki_peltier = parseFloat(values[1]);");
    client.println("      let Kd_peltier = parseFloat(values[2]);");
    client.println("      let setpointPeltier = parseFloat(values[3]);");  // Read SetpointPeltier
    client.println("      let setpointWB = parseFloat(values[4]);");  // Read SetpointWB
    client.println("      let waterBlockTemp = parseFloat(values[5]);");
    client.println("      let brainTemp = parseFloat(values[6]);");
    client.println("      let flowRateA = parseFloat(values[7]);");
    client.println("      let flowRateB = parseFloat(values[8]);");
    client.println("      let pumpVoltage = parseFloat(values[9]);");
    client.println("      let pumpCurrent = parseFloat(values[10]);");
    client.println("      let peltierVoltage = parseFloat(values[11]);");
    client.println("      let peltierCurrent = parseFloat(values[12]);");
    client.println("      let pressure1 = parseFloat(values[13]);");
    client.println("      let pressure2 = parseFloat(values[14]);");
    client.println("      let previousFreq = parseFloat(values[15]);");
    client.println("      let dacVoltage = parseFloat(values[16]);");

/*
        // Function to update the chart and value without refreshing the page
    client.println("function updateValue() {");
    client.println("  let xhr = new XMLHttpRequest();");
    client.println("  xhr.onreadystatechange = function() {");
    client.println("    if (xhr.readyState == 4 && xhr.status == 200) {");
    client.println("      let values = xhr.responseText.split(',');");
    client.println("      let Kp_peltier = parseFloat(values[0]);");
    client.println("      let Ki_peltier = parseFloat(values[1]);");
    client.println("      let Kd_peltier = parseFloat(values[2]);");
    client.println("      let setpointPeltier = parseFloat(values[3]);");
    client.println("      let setpointWB = parseFloat(values[4]);");
    client.println("      let steinhartIntraArray1 = parseFloat(values[5]);");
    client.println("      let steinhartIntraArray2 = parseFloat(values[6]);");
    client.println("      let steinhartIntraArray3 = parseFloat(values[7]);");
    client.println("      let steinhartIntraArray4 = parseFloat(values[8]);");
    client.println("      let steinhartExtraArray1 = parseFloat(values[9]);");
    client.println("      let steinhartExtraArray2 = parseFloat(values[10]);");
    client.println("      let steinhartExtraArray3 = parseFloat(values[11]);");
    client.println("      let steinhartSWB1 = parseFloat(values[12]);");
    client.println("      let steinhartSWB2 = parseFloat(values[13]);");
    client.println("      let steinhartSWB3 = parseFloat(values[14]);");
    client.println("      let steinhartEntrBWB1 = parseFloat(values[15]);");
    client.println("      let steinhartEntrBWB2 = parseFloat(values[16]);");
    client.println("      let steinhartEntrBWB3 = parseFloat(values[17]);");
    client.println("      let steinhartBWB1 = parseFloat(values[18]);");
    client.println("      let steinhartBWB2 = parseFloat(values[19]);");
    client.println("      let steinhartBWB3 = parseFloat(values[20]);");
    client.println("      let steinhartBWB4 = parseFloat(values[21]);");
    client.println("      let steinhartBWB5 = parseFloat(values[22]);");
    client.println("      let steinhartBWB6 = parseFloat(values[23]);");
    client.println("      let steinhartExitBWB1 = parseFloat(values[24]);");
    client.println("      let steinhartExitBWB2 = parseFloat(values[25]);");
    client.println("      let steinhartExitBWB3 = parseFloat(values[26]);");
    client.println("      let braintemp_atm = parseFloat(values[27]);");
    client.println("      let ExtraArrayTemp = parseFloat(values[28]);");
    client.println("      let SWBTemp_atm = parseFloat(values[29]);");
    client.println("      let EntrBWBTemp = parseFloat(values[30]);");
    client.println("      let BWBTemp = parseFloat(values[31]);");
    client.println("      let ExitBWBTemp = parseFloat(values[32]);");
    client.println("      let pressureApplied1 = parseFloat(values[33]);");
    client.println("      let pressureApplied2 = parseFloat(values[34]);");
    client.println("      let aFlow = parseFloat(values[35]);");
    client.println("      let bFlow = parseFloat(values[36]);");
    client.println("      let aTemperature = parseFloat(values[37]);");
    client.println("      let bTemperature = parseFloat(values[38]);");
    client.println("      let pumpVoltage = parseFloat(values[39]);");
    client.println("      let pumpCurrent = parseFloat(values[40]);");
    client.println("      let peltierVoltage = parseFloat(values[41]);");
    client.println("      let peltierCurrent = parseFloat(values[42]);");
    client.println("      let previousFreq = parseFloat(values[43]);");
    client.println("      let dacVoltage = parseFloat(values[44]);");
*/


    // Update the display
    client.println("      document.getElementById('proportionalValue').innerText = Kp_peltier;");
    client.println("      document.getElementById('integralValue').innerText = Ki_peltier;");
    client.println("      document.getElementById('derivativeValue').innerText = Kd_peltier;");
    client.println("      document.getElementById('setpointPeltierValue').innerText = setpointPeltier.toFixed(1);");  // Update display for SetpointPeltier
    client.println("      document.getElementById('setpointWBValue').innerText = setpointWB.toFixed(1);");  // Update display for SetpointWB
    client.println("      document.getElementById('SWBTempValue').innerText = waterBlockTemp.toFixed(1);");
    client.println("      document.getElementById('brainTempValue').innerText = brainTemp.toFixed(1);");
    client.println("      document.getElementById('aFlowValue').innerText = flowRateA.toFixed(1);");
    client.println("      document.getElementById('bFlowValue').innerText = flowRateB.toFixed(1);");
    client.println("      document.getElementById('pumpVoltageValue').innerText = pumpVoltage.toFixed(1);");
    client.println("      document.getElementById('pumpCurrentValue').innerText = pumpCurrent.toFixed(1);");
    client.println("      document.getElementById('peltierVoltageValue').innerText = peltierVoltage.toFixed(1);");
    client.println("      document.getElementById('peltierCurrentValue').innerText = peltierCurrent.toFixed(1);");
    client.println("      document.getElementById('pressure1Value').innerText = pressure1.toFixed(1);");
    client.println("      document.getElementById('pressure2Value').innerText = pressure2.toFixed(1);");
    client.println("      document.getElementById('previousFreqValue').innerText = previousFreq.toFixed(1);");
    client.println("      document.getElementById('dacVoltageValue').innerText = dacVoltage.toFixed(1);");
  

/*
    // Update the display
    client.println("      document.getElementById('proportionalValue').innerText = Kp_peltier;");
    client.println("      document.getElementById('integralValue').innerText = Ki_peltier;");
    client.println("      document.getElementById('derivativeValue').innerText = Kd_peltier;");
    client.println("      document.getElementById('setpointPeltierValue').innerText = SetpointPeltier.toFixed(1);");  // Update display for SetpointPeltier
    client.println("      document.getElementById('setpointWBValue').innerText = SetpointWB.toFixed(1);");  // Update display for SetpointWB");
    client.println("      document.getElementById('steinhartIntraArray1Value').innerText = steinhartIntraArray1.toFixed(1);");
    client.println("      document.getElementById('steinhartIntraArray2Value').innerText = steinhartIntraArray2.toFixed(1);");
    client.println("      document.getElementById('steinhartIntraArray3Value').innerText = steinhartIntraArray3.toFixed(1);");
    client.println("      document.getElementById('steinhartIntraArray4Value').innerText = steinhartIntraArray4.toFixed(1);");
    client.println("      document.getElementById('steinhartExtraArray1Value').innerText = steinhartExtraArray1.toFixed(1);");
    client.println("      document.getElementById('steinhartExtraArray2Value').innerText = steinhartExtraArray2.toFixed(1);");
    client.println("      document.getElementById('steinhartExtraArray3Value').innerText = steinhartExtraArray3.toFixed(1);");
    client.println("      document.getElementById('steinhartSWB1Value').innerText = steinhartSWB1.toFixed(1);");
    client.println("      document.getElementById('steinhartSWB2Value').innerText = steinhartSWB2.toFixed(1);");
    client.println("      document.getElementById('steinhartSWB3Value').innerText = steinhartSWB3.toFixed(1);");
    client.println("      document.getElementById('steinhartEntrBWB1Value').innerText = steinhartEntrBWB1.toFixed(1);");
    client.println("      document.getElementById('steinhartEntrBWB2Value').innerText = steinhartEntrBWB2.toFixed(1);");
    client.println("      document.getElementById('steinhartEntrBWB3Value').innerText = steinhartEntrBWB3.toFixed(1);");
    client.println("      document.getElementById('steinhartBWB1Value').innerText = steinhartBWB1.toFixed(1);");
    client.println("      document.getElementById('steinhartBWB2Value').innerText = steinhartBWB2.toFixed(1);");
    client.println("      document.getElementById('steinhartBWB3Value').innerText = steinhartBWB3.toFixed(1);");
    client.println("      document.getElementById('steinhartBWB4Value').innerText = steinhartBWB4.toFixed(1);");
    client.println("      document.getElementById('steinhartBWB5Value').innerText = steinhartBWB5.toFixed(1);");
    client.println("      document.getElementById('steinhartBWB6Value').innerText = steinhartBWB6.toFixed(1);");
    client.println("      document.getElementById('steinhartExitBWB1Value').innerText = steinhartExitBWB1.toFixed(1);");
    client.println("      document.getElementById('steinhartExitBWB2Value').innerText = steinhartExitBWB2.toFixed(1);");
    client.println("      document.getElementById('steinhartExitBWB3Value').innerText = steinhartExitBWB3.toFixed(1);");
    client.println("      document.getElementById('brainTempValue').innerText = braintemp_atm.toFixed(1);");
    client.println("      document.getElementById('extraArrayTempValue').innerText = ExtraArrayTemp.toFixed(1);");
    client.println("      document.getElementById('SWBTempAtmValue').innerText = SWBTemp_atm.toFixed(1);");
    client.println("      document.getElementById('entrBWBTempValue').innerText = EntrBWBTemp.toFixed(1);");
    client.println("      document.getElementById('BWBTempValue').innerText = BWBTemp.toFixed(1);");
    client.println("      document.getElementById('exitBWBTempValue').innerText = ExitBWBTemp.toFixed(1);");
    client.println("      document.getElementById('pressure1Value').innerText = pressureApplied1.toFixed(1);");
    client.println("      document.getElementById('pressure2Value').innerText = pressureApplied2.toFixed(1);");
    client.println("      document.getElementById('aFlowValue').innerText = aFlow.toFixed(1);");
    client.println("      document.getElementById('bFlowValue').innerText = bFlow.toFixed(1);");
    client.println("      document.getElementById('aTemperatureValue').innerText = aTemperature.toFixed(1);");
    client.println("      document.getElementById('bTemperatureValue').innerText = bTemperature.toFixed(1);");
    client.println("      document.getElementById('pumpVoltageValue').innerText = pumpVoltage.toFixed(1);");
    client.println("      document.getElementById('pumpCurrentValue').innerText = pumpCurrent.toFixed(1);");
    client.println("      document.getElementById('peltierVoltageValue').innerText = peltierVoltage.toFixed(1);");
    client.println("      document.getElementById('peltierCurrentValue').innerText = peltierCurrent.toFixed(1);");
    client.println("      document.getElementById('previousFreqValue').innerText = previousFreq.toFixed(1);");
    client.println("      document.getElementById('dacVoltageValue').innerText = dacVoltage.toFixed(1);");
*/



    // Push data to arrays
    client.println("      dataProportional.push(Kp_peltier);");
    client.println("      dataIntegral.push(Ki_peltier);");
    client.println("      dataDerivative.push(Kd_peltier);");
    client.println("      dataSetpointPeltier.push(setpointPeltier);");  // Store SetpointPeltier data
    client.println("      dataSetpointWB.push(setpointWB);");  // Store SetpointWB data
    client.println("      dataSWBTemp.push(waterBlockTemp);");
    client.println("      dataBrainTemp.push(brainTemp);");
    client.println("      dataAFlow.push(flowRateA);");
    client.println("      dataBFlow.push(flowRateB);");
    client.println("      dataPumpVoltage.push(pumpVoltage);");
    client.println("      dataPumpCurrent.push(pumpCurrent);");
    client.println("      dataPeltierVoltage.push(peltierVoltage);");
    client.println("      dataPeltierCurrent.push(peltierCurrent);");
    client.println("      dataPressure1.push(pressure1);");
    client.println("      dataPressure2.push(pressure2);");
    client.println("      dataPreviousFreq.push(previousFreq);");
    client.println("      dataDacVoltage.push(dacVoltage);");
    client.println("      labels.push(time++);");

/*

    // Push data to arrays
    client.println("      dataProportional.push(Kp_peltier);");
    client.println("      dataIntegral.push(Ki_peltier);");
    client.println("      dataDerivative.push(Kd_peltier);");
    client.println("      dataSetpointPeltier.push(SetpointPeltier);");  // Store SetpointPeltier data
    client.println("      dataSetpointWB.push(SetpointWB);");  // Store SetpointWB data
    client.println("      dataSteinhartIntraArray1.push(steinhartIntraArray1);");
    client.println("      dataSteinhartIntraArray2.push(steinhartIntraArray2);");
    client.println("      dataSteinhartIntraArray3.push(steinhartIntraArray3);");
    client.println("      dataSteinhartIntraArray4.push(steinhartIntraArray4);");
    client.println("      dataSteinhartExtraArray1.push(steinhartExtraArray1);");
    client.println("      dataSteinhartExtraArray2.push(steinhartExtraArray2);");
    client.println("      dataSteinhartExtraArray3.push(steinhartExtraArray3);");
    client.println("      dataSteinhartSWB1.push(steinhartSWB1);");
    client.println("      dataSteinhartSWB2.push(steinhartSWB2);");
    client.println("      dataSteinhartSWB3.push(steinhartSWB3);");
    client.println("      dataSteinhartEntrBWB1.push(steinhartEntrBWB1);");
    client.println("      dataSteinhartEntrBWB2.push(steinhartEntrBWB2);");
    client.println("      dataSteinhartEntrBWB3.push(steinhartEntrBWB3);");
    client.println("      dataSteinhartBWB1.push(steinhartBWB1);");
    client.println("      dataSteinhartBWB2.push(steinhartBWB2);");
    client.println("      dataSteinhartBWB3.push(steinhartBWB3);");
    client.println("      dataSteinhartBWB4.push(steinhartBWB4);");
    client.println("      dataSteinhartBWB5.push(steinhartBWB5);");
    client.println("      dataSteinhartBWB6.push(steinhartBWB6);");
    client.println("      dataSteinhartExitBWB1.push(steinhartExitBWB1);");
    client.println("      dataSteinhartExitBWB2.push(steinhartExitBWB2);");
    client.println("      dataSteinhartExitBWB3.push(steinhartExitBWB3);");
    client.println("      dataBrainTemp.push(braintemp_atm);");
    client.println("      dataExtraArrayTemp.push(ExtraArrayTemp);");
    client.println("      dataSWBTemp.push(SWBTemp_atm);");
    client.println("      dataEntrBWBTemp.push(EntrBWBTemp);");
    client.println("      dataBWBTemp.push(BWBTemp);");
    client.println("      dataExitBWBTemp.push(ExitBWBTemp);");
    client.println("      dataPressure1.push(pressureApplied1);");
    client.println("      dataPressure2.push(pressureApplied2);");
    client.println("      dataAFlow.push(aFlow);");
    client.println("      dataBFlow.push(bFlow);");
    client.println("      dataATemperature.push(aTemperature);");
    client.println("      dataBTemperature.push(bTemperature);");
    client.println("      dataPumpVoltage.push(pumpVoltage);");
    client.println("      dataPumpCurrent.push(pumpCurrent);");
    client.println("      dataPeltierVoltage.push(peltierVoltage);");
    client.println("      dataPeltierCurrent.push(peltierCurrent);");
    client.println("      dataPreviousFreq.push(previousFreq);");
    client.println("      dataDacVoltage.push(dacVoltage);");
    client.println("      labels.push(time++);");
*/




    // Limit data to 100 points
    client.println("      if (dataProportional.length > 100) { dataProportional.shift(); }");
    client.println("      if (dataIntegral.length > 100) { dataIntegral.shift(); }");
    client.println("      if (dataDerivative.length > 100) { dataDerivative.shift(); }");
    client.println("      if (dataSetpointPeltier.length > 100) { dataSetpointPeltier.shift(); }");  // Limit SetpointPeltier data
    client.println("      if (dataSetpointWB.length > 100) { dataSetpointWB.shift(); }");  // Limit SetpointWB data
    client.println("      if (dataSWBTemp.length > 100) { dataSWBTemp.shift(); }");
    client.println("      if (dataBrainTemp.length > 100) { dataBrainTemp.shift(); }");
    client.println("      if (dataAFlow.length > 100) { dataAFlow.shift(); }");
    client.println("      if (dataBFlow.length > 100) { dataBFlow.shift(); }");
    client.println("      if (dataPumpVoltage.length > 100) { dataPumpVoltage.shift(); }");
    client.println("      if (dataPumpCurrent.length > 100) { dataPumpCurrent.shift(); }");
    client.println("      if (dataPeltierVoltage.length > 100) { dataPeltierVoltage.shift(); }");
    client.println("      if (dataPeltierCurrent.length > 100) { dataPeltierCurrent.shift(); }");
    client.println("      if (dataPressure1.length > 100) { dataPressure1.shift(); }");
    client.println("      if (dataPressure2.length > 100) { dataPressure2.shift(); }");
    client.println("      if (dataPreviousFreq.length > 100) { dataPreviousFreq.shift(); }");
    client.println("      if (dataDacVoltage.length > 100) { dataDacVoltage.shift(); }");
    client.println("      if (labels.length > 100) { labels.shift(); }");



    // Update all charts
    client.println("      chart1.update();");
    client.println("      tempChart.update();");
    client.println("      flowRateChart.update();");
    client.println("      powerChart.update();");
    client.println("      pressureChart.update();");  
    client.println("      frequencyChart.update();");  
    client.println("      dacChart.update();");  
    client.println("    }");
    client.println("  };");
    client.println("  xhr.open('GET', '/getValues', true);");
    client.println("  xhr.send();");
    client.println("}");
    
    // Function to handle AJAX form submission to update Proportional, Integral, or Derivative value
    client.println("function submitForm(event, variable) {");
    client.println("  event.preventDefault();  // Prevent page refresh");
    client.println("  var xhr = new XMLHttpRequest();");
    client.println("  var newVal = document.getElementById('newVal' + variable).value;");
    client.println("  xhr.open('GET', '/update' + variable + '?newVal=' + newVal, true);");
    client.println("  xhr.onreadystatechange = function() {");
    client.println("    if (xhr.readyState == 4 && xhr.status == 200) {");
    client.println("      console.log(variable + ' updated to: ' + newVal);");
    client.println("    }");
    client.println("  };");
    client.println("  xhr.send();");
    client.println("}");

    // Function to download CSV
    client.println("function downloadCSV() {");
    client.println("  let csvContent = 'data:text/csv;charset=utf-8,Time,Proportional,Integral,Derivative,Scalp Water Block Temp,Brain Temp,Flow Rate A,Flow Rate B\\n';");
    client.println("  for (let i = 0; i < dataProportional.length; i++) {");
    client.println("    csvContent += labels[i] + ',' + dataProportional[i] + ',' + dataIntegral[i] + ',' + dataDerivative[i] + ',' + dataSWBTemp[i] + ',' + dataBrainTemp[i] + ',' + dataAFlow[i] + ',' + dataBFlow[i] + '\\n';");
    client.println("  }");
    client.println("  var encodedUri = encodeURI(csvContent);");
    client.println("  var link = document.createElement('a');");
    client.println("  link.setAttribute('href', encodedUri);");
    client.println("  link.setAttribute('download', 'data.csv');");
    client.println("  document.body.appendChild(link);");
    client.println("  link.click();");
    client.println("}");

    // Function to Reset Arduino
    client.println("function sendSignal() {");
    client.println("fetch('/triggerDigitalSignal')");
    client.println(".then(response => response.text())");
    client.println(".then(data => console.log(data))");
    client.println(".catch(error => console.error('Error:', error));");
    client.println("}");

    client.println("setInterval(updateValue, 1000);");  
    client.println("</script>");
    client.println("</body></html>");

    client.stop();

  } else if (client) {
    // Clean up any stale clients
    client.stop(); }
}

#endif // WEBDASHBOARD_H