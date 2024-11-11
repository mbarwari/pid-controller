#ifndef WEBDASHBOARD_H
#define WEBDASHBOARD_H

#include <WiFi.h>
#include "chartjs.h"  // Include the chart.js content

// Error message declaration
String currentErrorMessage = ""; // Initialize to an empty string

// Function prototypes
void setupWebDashboard(WiFiServer& server);
void handleWebRequests(WiFiServer& server);

// References to the main script's variables
extern double Kp_peltier, Ki_peltier, Kd_peltier;
extern double SetpointPeltier, SetpointWB;
extern float WBTemp_atm, braintemp_atm;
extern float aFlow, bFlow;
extern float pumpVoltage, pumpCurrent;
extern float peltierVoltage, peltierCurrent;
extern float pressureApplied1, pressureApplied2;
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
      client.print(WBTemp_atm);
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

    // Display the current values
    client.println("<p>Proportional: <span id='proportionalValue'>0</span></p>");
    client.println("<p>Integral: <span id='integralValue'>0</span></p>");
    client.println("<p>Derivative: <span id='derivativeValue'>0</span></p>");
    client.println("<p>Setpoint Peltier: <span id='setpointPeltierValue'>" + String(SetpointPeltier) + "</span></p>");  // Display SetpointPeltier
    client.println("<p>Setpoint WB: <span id='setpointWBValue'>" + String(SetpointWB) + "</span></p>");  // Display SetpointWB
    client.println("<p>Water Bottle Temperature: <span id='WBTempValue'>0.0</span></p>");
    client.println("<p>Brain Temperature: <span id='brainTempValue'>0.0</span></p>");
    client.println("<p>Flow Rate A: <span id='aFlowValue'>0.0</span></p>");
    client.println("<p>Flow Rate B: <span id='bFlowValue'>0.0</span></p>");
    client.println("<p>Pump Voltage: <span id='pumpVoltageValue'>0.0</span></p>");
    client.println("<p>Pump Current: <span id='pumpCurrentValue'>0.0</span></p>");
    client.println("<p>Peltier Voltage: <span id='peltierVoltageValue'>0.0</span></p>");
    client.println("<p>Peltier Current: <span id='peltierCurrentValue'>0.0</span></p>");
    client.println("<p>Pressure Applied 1: <span id='pressure1Value'>0.0</span></p>");
    client.println("<p>Pressure Applied 2: <span id='pressure2Value'>0.0</span></p>");
    client.println("<p>Frequency output: <span id='previousFreqValue'>0.0</span></p>");
    client.println("<p>DAC Voltage: <span id='dacVoltageValue'>0.0</span></p>");

    // Canvas for charts
    client.println("<canvas id='proportionalChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='temperatureChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='flowRateChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='powerChart' style='width: 200px; height: 100px;'></canvas>");
    client.println("<canvas id='pressureChart' style='width: 200px; height: 100px;'></canvas>");

    // Button to download CSV
    client.println("<button onclick='downloadCSV()'>Download CSV</button>");

    // JavaScript to update the value, the chart, and download data as CSV
    client.println("<script>");
    client.println("let dataProportional = [];");
    client.println("let dataIntegral = [];");
    client.println("let dataDerivative = [];");
    client.println("let dataSetpointPeltier = [];");
    client.println("let dataSetpointWB = [];");
    client.println("let dataWBTemp = [];");
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
    client.println("        label: 'Water Block Temp',");
    client.println("        data: dataWBTemp,");
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
    client.println("        y: { title: { display: true, text: 'Temperature (°C)' } }");
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
    client.println("        xAxes: [{");
    client.println("          type: 'linear',");
    client.println("          position: 'bottom'");
    client.println("        }]");
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
    client.println("      let waterBottleTemp = parseFloat(values[5]);");
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

    // Update the display
    client.println("      document.getElementById('proportionalValue').innerText = Kp_peltier;");
    client.println("      document.getElementById('integralValue').innerText = Ki_peltier;");
    client.println("      document.getElementById('derivativeValue').innerText = Kd_peltier;");
    client.println("      document.getElementById('setpointPeltierValue').innerText = setpointPeltier.toFixed(1);");  // Update display for SetpointPeltier
    client.println("      document.getElementById('setpointWBValue').innerText = setpointWB.toFixed(1);");  // Update display for SetpointWB
    client.println("      document.getElementById('WBTempValue').innerText = waterBottleTemp.toFixed(1);");
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

    // Push data to arrays
    client.println("      dataProportional.push(Kp_peltier);");
    client.println("      dataIntegral.push(Ki_peltier);");
    client.println("      dataDerivative.push(Kd_peltier);");
    client.println("      dataSetpointPeltier.push(setpointPeltier);");  // Store SetpointPeltier data
    client.println("      dataSetpointWB.push(setpointWB);");  // Store SetpointWB data
    client.println("      dataWBTemp.push(waterBottleTemp);");
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

    // Limit data to 100 points
    client.println("      if (dataProportional.length > 100) { dataProportional.shift(); }");
    client.println("      if (dataIntegral.length > 100) { dataIntegral.shift(); }");
    client.println("      if (dataDerivative.length > 100) { dataDerivative.shift(); }");
    client.println("      if (dataSetpointPeltier.length > 100) { dataSetpointPeltier.shift(); }");  // Limit SetpointPeltier data
    client.println("      if (dataSetpointWB.length > 100) { dataSetpointWB.shift(); }");  // Limit SetpointWB data
    client.println("      if (dataWBTemp.length > 100) { dataWBTemp.shift(); }");
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
    client.println("  let csvContent = 'data:text/csv;charset=utf-8,Time,Proportional,Integral,Derivative,Water Block Temp,Brain Temp,Flow Rate A,Flow Rate B\\n';");
    client.println("  for (let i = 0; i < dataProportional.length; i++) {");
    client.println("    csvContent += labels[i] + ',' + dataProportional[i] + ',' + dataIntegral[i] + ',' + dataDerivative[i] + ',' + dataWBTemp[i] + ',' + dataBrainTemp[i] + ',' + dataAFlow[i] + ',' + dataBFlow[i] + '\\n';");
    client.println("  }");
    client.println("  var encodedUri = encodeURI(csvContent);");
    client.println("  var link = document.createElement('a');");
    client.println("  link.setAttribute('href', encodedUri);");
    client.println("  link.setAttribute('download', 'data.csv');");
    client.println("  document.body.appendChild(link);");
    client.println("  link.click();");
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

