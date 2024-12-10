//index_html
const char index_html[] PROGMEM = R"rawliteral(
<!doctype html><html><head><title>ESP32 Modbus</title><meta name=viewport content="width=device-width,initial-scale=1"><style>body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Oxygen,Ubuntu,Cantarell,'Open Sans','Helvetica Neue',sans-serif;margin:0;padding:0;display:flex;justify-content:center;align-items:center;min-height:100vh;background-color:#1a1a1a;color:#e0e0e0;transition:background-color .3s,color .3s}main{width:90%;max-width:800px}h1,h2,h3{font-weight:400;margin-bottom:.5em}h1{font-size:2em}h2{font-size:1.5em}h3{font-size:1.2em}p,div{margin-bottom:1em}form{margin-bottom:1em}input[type=range],input[type=text],select,button{font-size:1em;padding:.5em;border:1px solid #333;border-radius:4px;background-color:#333;color:#e0e0e0;transition:background-color .3s,color .3s}input[type=range]{width:100%}button{background-color:#007bff;color:#fff;cursor:pointer;border:none;padding:.5em 1em;transition:background-color .3s}button:hover{background-color:#0056b3}table{width:100%;border-collapse:collapse}td,th{border:1px solid #444;padding:.5em;text-align:left}th{background-color:#222;font-weight:400}ul{list-style-type:none;padding:0}li{margin-bottom:.3em}.row{display:flex;flex-wrap:wrap;gap:1em}.column{flex:1;min-width:300px}body.light-mode{background-color:#f8f9fa;color:#333}body.light-mode input[type=range],body.light-mode input[type=text],body.light-mode select,body.light-mode button{border:1px solid #ddd;background-color:#fff;color:#333}body.light-mode th{background-color:#f2f2f2}body.light-mode td,body.light-mode th{border:1px solid #ddd}.mode-switch{position:fixed;top:1rem;right:1rem}div>div{display:flex;justify-content:space-between;align-items:center}</style><script src="script.js"></script></head><body class=dark-mode><main><label class=mode-switch><input type=checkbox id=darkModeSwitch>Toggle Light Mode</label><h1>ESP32 Modbus</h1><h2>Update Registers</h2><div class=row><div class=column><form id=updateRegistersForm><div><div><label for=gridChargeCurrent>Grid Charge Current (A):</label><input type=range id=gridChargeCurrent name=gridChargeCurrent min=0 max=40 step=5 value=0 disabled><span id=gridChargeCurrentValue>0 A</span></div><button type=button id=updateGridChargeCurrent disabled>Update</button></div><div><div><label for=outputPriority>Priority:</label><select id=outputPriority name=outputPriority disabled><option value=0>Solar</option><option value=1>Utility</option><option value=2>Solar-Battery</option></select></div><button type=button id=updateOutputPriority disabled>Update</button></div><div><div><label for=e120Slider>PV Charge Current (A):</label><input type=range id=e120Slider name=e120 min=0 max=100 step=5 value=0 disabled><span id=e120Value>0 A</span></div><button type=button id=updatePVChargeCurrent disabled>Update</button></div><div><div><label for=e20aSlider>Total Max Charge Limit (A):</label><input type=range id=e20aSlider name=e20a min=0 max=100 step=5 value=0 disabled><span id=e20aValue>0 A</span></div><button type=button id=updateTotalMaxCharge disabled>Update</button></div></form></div><div class=column><h2>Setting Responses</h2><div id=settingResponses></div></div></div><h2>Status</h2><table id=batteryTable><thead><tr><th>Name</th><th>Value</th><th>Description</th></tr></thead><tbody></tbody></table><h2>Read Register</h2><form action=/read_modbus_register><input name=address placeholder=e.g., 0xE120><input type=submit value=Read></form><div id=modbusResult>Result:</div><h2>Last Queries</h2><div id=lastQueries></div><h2>Sysinfo</h2><p id=buildInfo></p><p id=uptimeInfo></p><h2>Memory Usage</h2><div id=memoryUsage></div><h2>Settings</h2><div><label for=slaveAddress>Modbus Slave:</label><select id=slaveAddress></select><button id=updateSlaveAddressButton>Update</button></div><h3>BLE Name</h3><form method=POST action=/change_blename><input name=blename><input type=submit value=Change></form><h3>JBD BMS MAC Address</h3><form id=setJbdMacForm><input type=text id=jbdMac name=mac placeholder=e.g., 70:3e:97:07:c0:3e required><button type=button id=setJbdMacButton>Set MAC</button></form><h2>Control</h2><form id=restartOtaForm><label for=otaConfirm>Restart OTA:</label><input id=otaConfirm name=confirm placeholder=doit required><button type=submit>Restart</button></form><form id=restartEspForm><label for=espConfirm>Restart ESP32:</label><input id=espConfirm name=confirm placeholder=doit required><button type=submit>Restart</button></form><h1>File Management</h1><p><a href="/files">View Files</a></p><h2>Upload File</h2><form method=POST action=/upfile enctype=multipart/form-data><input type=file name=data><input type=submit name=upload value=Upload title=Upload File></form><p>After clicking upload it will take some time for the file to firstly upload and then be written to SPIFFS, there is no indicator that the upload began. Please be patient.</p><p>Once uploaded, you can view the new file by clicking 'View Files'.</p><p>If a file does not appear, it will be because the file was too big, or had unusual characters in the file name (like spaces).</p><p>You can see the progress of the upload by watching the serial output.</p></main></body></html>
)rawliteral";