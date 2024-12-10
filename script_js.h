// script_js.h
const char script_js[] PROGMEM = R"rawliteral(
document.addEventListener("DOMContentLoaded",function(){const e=document.body,t=document.getElementById("darkModeSwitch"),n=document.getElementById("batteryTable"),o=e=>{e.classList.toggle("light-mode",!localStorage.setItem("darkMode",e))},l=()=>fetch("/last_queries").then(e=>e.text()).then(e=>lastQueries.innerHTML=e),d=()=>fetch("/get_setting_responses").then(e=>e.json()).then(e=>{let t="<ul>";for(let n of e)t+=`<li><strong>${n.setting}:</strong> ${n.response}</li>`;settingResponses.innerHTML=t+="</ul>"}),a=()=>fetch("/uptime").then(e=>e.text()).then(e=>uptimeInfo.textContent="Uptime: "+e),i=()=>fetch("/get_build_info").then(e=>e.text()).then(e=>buildInfo.textContent="Build Info: "+e),r=()=>fetch("/battery_status").then(e=>e.json()).then(e=>{const t=n.querySelector("tbody");for(let n of e){let o=Array.from(t.rows).find(e=>e.cells[0].textContent===n.name);o||(o=t.insertRow(),o.insertCell(0).textContent=n.name,o.insertCell(1),o.insertCell(2).textContent=n.description),o.cells[1].textContent=null===n.value?"N/A":n.unit?(n.isInteger?n.value:n.value.toFixed(2))+" "+n.unit:n.isInteger?n.value:n.value.toFixed(2),n.description!==o.cells[2].textContent&&(o.cells[2].textContent=n.description)}}).catch(e=>console.error("Battery status error:",e)),c=()=>{const e=document.getElementById("slaveAddress");e?fetch("/set_modbus_slave",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"slaveAddress="+e.value}).then(e=>e.text()).then(e=>alert("Update successful: "+e)).catch(e=>alert("Slave address error: "+e.message)):console.error("Slave address not found")},s=(e,t)=>document.getElementById(t).textContent=document.getElementById(e).value+" A",u=(e,t)=>fetch("/update_registers",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:`${e}=${t}`}).then(e=>e.text()).then(e=>{alert(e),d()}).catch(e=>alert("Setting update error: "+e.message)),m=e=>{e.preventDefault(),fetch("/change_blename",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"blename="+encodeURIComponent(e.target.querySelector('input[name="blename"]').value)}).then(e=>e.text()).then(e=>{alert(e),location.reload()}).catch(e=>alert("BLE name change failed: "+e.message))},f=e=>{e.preventDefault(),fetch("/read_modbus_register?address="+encodeURIComponent(e.target.querySelector('input[name="address"]').value)).then(e=>e.text()).then(e=>{modbusResult.textContent="Result: "+e,l()}).catch(e=>modbusResult.textContent="Error: "+e.message)},g=e=>{e.preventDefault();const t=e.target.confirm.value;"doit"===t?fetch("/restart_ota",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"confirm=doit"}).then(e=>e.text()).then(e=>alert(e)):alert('Confirmation must be "doit"')},p=e=>{e.preventDefault();const t=e.target.confirm.value;"doit"===t?fetch("/restart_esp",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"confirm=doit"}).then(e=>e.text()).then(e=>alert(e)):alert('Confirmation must be "doit"')},h=()=>fetch("/get_memory_usage").then(e=>e.text()).then(e=>{document.getElementById("memoryUsage").textContent=e}).catch(e=>console.error("Error fetching memory usage:",e)),b=()=>{const e=document.getElementById("jbdMac").value;fetch("/set_jbd_mac",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"mac="+encodeURIComponent(e)}).then(e=>e.text()).then(e=>alert(e)).catch(e=>{console.error("Error:",e),alert("Failed to set MAC address. Check console for details.")})};fetch("/current_settings").then(e=>e.json()).then(e=>{document.getElementById("gridChargeCurrent").value=e.gridChargeCurrentLimit,document.getElementById("gridChargeCurrentValue").textContent=e.gridChargeCurrentLimit+" A",document.getElementById("outputPriority").value=e.outputPriority,document.getElementById("e120Slider").value=e.pvChargeCurrent,document.getElementById("e120Value").textContent=e.pvChargeCurrent+" A",document.getElementById("e20aSlider").value=e.totalMaxChargeLimit,document.getElementById("e20aValue").textContent=e.totalMaxChargeLimit+" A",["gridChargeCurrent","outputPriority","e120Slider","e20aSlider"].forEach(e=>document.getElementById(e).disabled=!1),["updateGridChargeCurrent","updateOutputPriority","updatePVChargeCurrent","updateTotalMaxCharge"].forEach(e=>document.getElementById(e).disabled=!1)}).catch(e=>console.error("Error updating settings:",e)),document.getElementById("gridChargeCurrent").addEventListener("input",()=>s("gridChargeCurrent","gridChargeCurrentValue")),document.getElementById("e120Slider").addEventListener("input",()=>s("e120Slider","e120Value")),document.getElementById("e20aSlider").addEventListener("input",()=>s("e20aSlider","e20aValue")),document.getElementById("updateGridChargeCurrent").addEventListener("click",()=>u("gridChargeCurrent",document.getElementById("gridChargeCurrent").value)),document.getElementById("updateOutputPriority").addEventListener("click",()=>u("outputPriority",document.getElementById("outputPriority").value)),document.getElementById("updatePVChargeCurrent").addEventListener("click",()=>u("e120",document.getElementById("e120Slider").value)),document.getElementById("updateTotalMaxCharge").addEventListener("click",()=>u("e20a",document.getElementById("e20aSlider").value)),document.getElementById("restartOtaForm").addEventListener("submit",g),document.getElementById("restartEspForm").addEventListener("submit",p),fetch("/get_modbus_slave").then(e=>e.text()).then(e=>{const t=document.getElementById("slaveAddress");if(t){for(let n=1;n<=254;n++){const e=document.createElement("option");e.value=n,e.text=n,t.appendChild(e)}t.value=e}}).catch(e=>console.error("Error fetching modbus slave:",e)),document.getElementById("updateSlaveAddressButton").addEventListener("click",c),t.addEventListener("change",()=>{o(e)}),setInterval(r,5e3),setInterval(d,1e4),setInterval(a,6e4),setInterval(i,3e5),fetch("/get_blename").then(e=>e.text()).then(e=>document.querySelector('input[name="blename"]').value=e),document.querySelector('form[action="/change_blename"]').addEventListener("submit",m),document.querySelector('form[action="/read_modbus_register"]').addEventListener("submit",f),l(),d(),a(),i(),r(),h(),setInterval(h,3e5),document.getElementById("setJbdMacButton").addEventListener("click",b)});
)rawliteral";