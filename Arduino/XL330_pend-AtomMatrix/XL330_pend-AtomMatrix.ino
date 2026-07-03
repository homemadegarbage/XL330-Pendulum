#include <M5Atom.h>
#include <Kalman.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Dynamixel2Arduino.h>

WebServer server(80);

const char ssid[] = "XL330-pend";
const char pass[] = "password";

const IPAddress ip(192,168,40,1);
const IPAddress subnet(255,255,255,0);


#define DXL_SERIAL   Serial1
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL);
using namespace ControlTableItem;


float Kp = 0.64, Kd = 1.4, Kw = 0.8, Ki = 1.0;
float Kvel = 0.12;
float Kroll = 2.0;
float Kyaw = 0.02; 
float offset = 0;
int Delay = 0;



float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float theta = 0.0;

float kalAngle, kalAngleDot;
Kalman kalman;

float theta0_hat=0.0;
float vel=0.0;

float Mbal = 0.0, M0 = 0.0, M1 = 0.0;
float Tturn = 0.0;
float omegaCmd = 0.0, velCmd =0.0, velCmdFiler =0.0;
int webStickX = 0, webStickY = 0;
unsigned long lastWebStickMs = 0;



unsigned long oldTime = 0, loopTime, nowTime;
float dt;

int GetUP = 0;

Preferences preferences;


//   IMU
void get_theta(){
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  theta  = atan2(accY, -accZ) * RAD_TO_DEG + offset;
}

void get_gyro(){
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  gyroX = -gyroX;
}



//ブラウザ表示
void handleRoot() {
  String temp ="<!DOCTYPE html> \n<html lang=\"ja\">";
  temp +="<head>";
  temp +="<meta charset=\"utf-8\">";
  temp +="<title>XL330-pend</title>";
  temp +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp +="<style>";
  temp +=".container{max-width:520px;margin:auto;text-align:center;font-size:1.2rem;font-family:sans-serif;}";
  temp +=".topbar{display:flex;justify-content:flex-end;margin:16px 8px;}";
  temp +=".nav{display:inline-block;padding:10px 14px;border:1px solid #333;border-radius:6px;color:#111;text-decoration:none;font-weight:bold;}";
  temp +=".joy{position:relative;width:min(78vw,320px);height:min(78vw,320px);margin:28px auto 16px;border:2px solid #333;border-radius:50%;background:#f2f2f2;touch-action:none;}";
  temp +=".stick{position:absolute;width:34%;height:34%;left:33%;top:33%;border-radius:50%;background:#333;box-shadow:0 4px 14px rgba(0,0,0,.3);}";
  temp +=".axis{display:flex;justify-content:center;gap:16px;margin:12px 0 20px;}";
  temp +=".axis span{display:inline-block;min-width:92px;padding:8px;border:1px solid #ccc;border-radius:6px;}";
  temp +="button{width:120px;height:44px;font-weight:bold;margin:8px;border-radius:6px;border:1px solid #333;background:white;}";
  temp +="</style>";
  temp +="</head>";
  temp +="<body>";
  temp +="<div class=\"container\">";
  temp +="<div class=\"topbar\"><a class=\"nav\" href=\"/params\">&#12497;&#12521;&#12513;&#12540;&#12479;&#35519;&#25972;</a></div>";
  temp +="<h2>Joystick</h2>";
  temp +="<div id=\"joy\" class=\"joy\"><div id=\"stick\" class=\"stick\"></div></div>";
  temp +="<div class=\"axis\"><span>X:<b id=\"xv\">0</b></span><span>Y:<b id=\"yv\">0</b></span></div>";
  temp +="<button id=\"stop\">STOP</button>";
  temp +="<script>";
  temp +="const joy=document.getElementById('joy'),stick=document.getElementById('stick'),xv=document.getElementById('xv'),yv=document.getElementById('yv'),stop=document.getElementById('stop');";
  temp +="let active=false,x=0,y=0,last=0;";
  temp +="function setStick(nx,ny){x=Math.max(-127,Math.min(127,Math.round(nx)));y=Math.max(-127,Math.min(127,Math.round(ny)));const r=joy.clientWidth/2;stick.style.transform=`translate(${x/127*r*0.66}px,${y/127*r*0.66}px)`;xv.textContent=x;yv.textContent=y;}";
  temp +="function send(force=false){const now=Date.now();if(!force&&now-last<50)return;last=now;fetch(`/joy?x=${x}&y=${y}`,{cache:'no-store'}).catch(()=>{});}";
  temp +="function point(e){const b=joy.getBoundingClientRect();const cx=b.left+b.width/2,cy=b.top+b.height/2;let dx=e.clientX-cx,dy=e.clientY-cy;const max=b.width*0.33;const d=Math.hypot(dx,dy);if(d>max){dx=dx/d*max;dy=dy/d*max;}setStick(dx/max*127,dy/max*127);send();}";
  temp +="function reset(){active=false;setStick(0,0);send(true);}";
  temp +="joy.addEventListener('pointerdown',e=>{active=true;joy.setPointerCapture(e.pointerId);point(e);});";
  temp +="joy.addEventListener('pointermove',e=>{if(active)point(e);});";
  temp +="joy.addEventListener('pointerup',reset);joy.addEventListener('pointercancel',reset);";
  temp +="stop.addEventListener('click',reset);setInterval(()=>send(),120);";
  temp +="setStick(0,0);send(true);";
  temp +="</script>";
  temp +="</div>";
  temp +="</body>";
  server.send(200, "text/HTML", temp);
}

void handleParams() {
  String temp ="<!DOCTYPE html> \n<html lang=\"ja\">";
  temp +="<head>";
  temp +="<meta charset=\"utf-8\">";
  temp +="<title>XL330-pend</title>";
  temp +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp +="<style>";
  temp +=".container{";
  temp +="  max-width: 500px;";
  temp +="  margin: auto;";
  temp +="  text-align: center;";
  temp +="  font-size: 1.2rem;";
  temp +="}";
  temp +="span,.pm{";
  temp +="  display: inline-block;";
  temp +="  border: 1px solid #ccc;";
  temp +="  width: 50px;";
  temp +="  height: 30px;";
  temp +="  vertical-align: middle;";
  temp +="  margin-bottom: 20px;";
  temp +="}";
  temp +="span{";
  temp +="  width: 120px;";
  temp +="}";
  temp +="button{";
  temp +="  width: 100px;";
  temp +="  height: 40px;";
  temp +="  font-weight: bold;";
  temp +="  margin-bottom: 20px;";
  temp +="}";
  temp +="button.on{ background:lime; color:white; }";
  temp +=".column-3{ max-width:330px; margin:auto; text-align:center; display:flex; justify-content:space-between; flex-wrap:wrap; }";
  temp +="</style>";
  temp +="</head>";
  
  temp +="<body>";
  temp +="<div class=\"container\">";
  temp +="<a class=\"pm\" style=\"width:120px;height:34px;line-height:34px;text-decoration:none;color:#111;\" href=\"/\">&#25147;&#12427;</a><br>";

  //Kp
  temp +="Kp<br>";
  temp +="<a class=\"pm\" href=\"/KpM\">-</a>";
  temp +="<span>" + String(Kp) + "</span>";
  temp +="<a class=\"pm\" href=\"/KpP\">+</a><br>";

  //Kd
  temp +="Kd<br>";
  temp +="<a class=\"pm\" href=\"/KdM\">-</a>";
  temp +="<span>" + String(Kd) + "</span>";
  temp +="<a class=\"pm\" href=\"/KdP\">+</a><br>";

  //Kw
  temp +="Kw<br>";
  temp +="<a class=\"pm\" href=\"/KwM\">-</a>";
  temp +="<span>" + String(Kw) + "</span>";
  temp +="<a class=\"pm\" href=\"/KwP\">+</a><br>";

  //Ki
  temp +="Ki<br>";
  temp +="<a class=\"pm\" href=\"/KiM\">-</a>";
  temp +="<span>" + String(Ki) + "</span>";
  temp +="<a class=\"pm\" href=\"/KiP\">+</a><br>";

  //Kyaw
  temp +="Kyaw<br>";
  temp +="<a class=\"pm\" href=\"/KyawM\">-</a>";
  temp +="<span>" + String(Kyaw) + "</span>";
  temp +="<a class=\"pm\" href=\"/KyawP\">+</a><br>";

  //Kvel
  temp +="Kvel<br>";
  temp +="<a class=\"pm\" href=\"/KvelM\">-</a>";
  temp +="<span>" + String(Kvel) + "</span>";
  temp +="<a class=\"pm\" href=\"/KvelP\">+</a><br>";

  //Kroll
  temp +="Kroll<br>";
  temp +="<a class=\"pm\" href=\"/KrollM\">-</a>";
  temp +="<span>" + String(Kroll) + "</span>";
  temp +="<a class=\"pm\" href=\"/KrollP\">+</a><br>";

  //Delay
  temp +="Delay<br>";
  temp +="<a class=\"pm\" href=\"/DelayM\">-</a>";
  temp +="<span>" + String(Delay) + "</span>";
  temp +="<a class=\"pm\" href=\"/DelayP\">+</a><br>";

  //offset
  temp +="offset<br>";
  temp +="<a class=\"pm\" href=\"/offsetM\">-</a>";
  temp +="<span>" + String(offset) + "</span>";
  temp +="<a class=\"pm\" href=\"/offsetP\">+</a><br>";
  

  temp +="</div>";
  temp +="</body>";
  server.send(200, "text/HTML", temp);
}

void handleJoy() {
  if(server.hasArg("x")) webStickX = constrain(server.arg("x").toInt(), -127, 127);
  if(server.hasArg("y")) webStickY = constrain(server.arg("y").toInt(), -127, 127);
  lastWebStickMs = millis();
  server.send(200, "text/plain", "OK");
}


void KpM() {
  if(Kp >= -300.0){
    Kp -= 0.01;
    preferences.putFloat("Kp", Kp);
  }
  handleParams();
}

void KpP() {
  if(Kp <= 300){
    Kp += 0.01;
    preferences.putFloat("Kp", Kp);
  }
  handleParams();
}

void KdM() {
  if(Kd >= -300.0){
    Kd -= 0.01;
    preferences.putFloat("Kd", Kd);
  }
  handleParams();
}

void KdP() {
  if(Kd <= 300){
    Kd += 0.01;
    preferences.putFloat("Kd", Kd);
  }
  handleParams();
}

void KwM() {
  if(Kw >= -30){
    Kw -= 0.1;
    preferences.putFloat("Kw", Kw);
  }
  handleParams();
}

void KwP() {
  if(Kw <= 30){
    Kw += 0.1;
    preferences.putFloat("Kw", Kw);
  }
  handleParams();
}

void KiM() {
  if(Ki >= -10){
    Ki -= 0.1;
    preferences.putFloat("Ki", Ki);
  }
  handleParams();
}
void KiP() {
  if(Ki <= 10){
    Ki += 0.1;
    preferences.putFloat("Ki", Ki);
  }
  handleParams();
}

void KyawM() {
  if(Kyaw >= -10){
    Kyaw -= 0.01;
    preferences.putFloat("Kyaw", Kyaw);
  }
  handleParams();
}
void KyawP() {
  if(Kyaw <= 10){
    Kyaw += 0.01;
    preferences.putFloat("Kyaw", Kyaw);
  }
  handleParams();
}

void KvelM() {
  if(Kvel > 0){
    Kvel -= 0.01;
    preferences.putFloat("Kvel", Kvel);
  }
  handleParams();
}
void KvelP() {
  if(Kvel <= 30){
    Kvel += 0.01;
    preferences.putFloat("Kvel", Kvel);
  }
  handleParams();
}

void KrollM() {
  if(Kroll > 0){
    Kroll -= 0.2;
    preferences.putFloat("Kroll", Kroll);
  }
  handleParams();
}
void KrollP() {
  if(Kroll <= 30){
    Kroll += 0.2;
    preferences.putFloat("Kroll", Kroll);
  }
  handleParams();
}

void DelayM() {
  if(Delay > 0){
    Delay -= 1;
    preferences.putInt("Delay", Delay);
  }
  handleParams();
}
void DelayP() {
  if(Delay <= 30){
    Delay += 1;
    preferences.putInt("Delay", Delay);
  }
  handleParams();
}

void offsetM() {
  if(offset >= -10.0){
    offset -= 0.1;
    preferences.putFloat("offset", offset);
  }
  handleParams();
}
void offsetP() {
  if(offset <= 10.0){
    offset += 0.1;
    preferences.putFloat("offset", offset);
  }
  handleParams();
}


// Core0
void core0(void *){
  Serial.println("-----------------------------");
  uint8_t btmac[6];
  esp_read_mac(btmac, ESP_MAC_BT);
  Serial.printf("[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", btmac[0], btmac[1], btmac[2], btmac[3], btmac[4], btmac[5]);


  M5.dis.setBrightness(20);

  WiFi.softAP(ssid,pass);
  delay(100);
  WiFi.softAPConfig(ip,ip,subnet);

  server.on("/", handleRoot); 
  server.on("/params", handleParams);
  server.on("/joy", handleJoy);

  server.on("/KpP", KpP);
  server.on("/KpM", KpM);
  server.on("/KdP", KdP);
  server.on("/KdM", KdM);
  server.on("/KwP", KwP);
  server.on("/KwM", KwM);
  server.on("/KiP", KiP);
  server.on("/KiM", KiM);
  server.on("/KyawP", KyawP);
  server.on("/KyawM", KyawM);

  server.on("/KvelP", KvelP);
  server.on("/KvelM", KvelM);
  server.on("/KrollP", KrollP);
  server.on("/KrollM", KrollM);
  
  server.on("/DelayM", DelayM);
  server.on("/DelayP", DelayP);
  
  server.on("/offsetM", offsetM);
  server.on("/offsetP", offsetP);
  
  server.begin();

  for(;;){
    server.handleClient();

    if (millis() - lastWebStickMs < 500) {
      omegaCmd = webStickX * Kroll;
      velCmd = webStickY * Kvel;
    }else{
      omegaCmd = 0.0;
      velCmd = 0.0;
    }

    //LED表示
    M5.dis.clear();
      
    if(kalAngle > 20.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i, 0xff0000);
      }
    }else if(kalAngle <= 20.0 && kalAngle > 12.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i, 0x0000ff);
      }
    }else if(kalAngle <= 12.0 && kalAngle > 4.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 5, 0x0000ff);
      }
    }else if(kalAngle <= 4.0 && kalAngle > 1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 10, 0x0000ff);
      }
    }else if(abs(kalAngle) <= 1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 10, 0x00ff00);
      }
    }else if(kalAngle >= -4.0 && kalAngle < -1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 10, 0x0000ff);
      }
    }else if(kalAngle >= -12.0 && kalAngle < -4.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 15, 0x0000ff);
      }
    }else if(kalAngle >= -20.0 && kalAngle < -12.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 20, 0x0000ff);
      }
    }else if(kalAngle < -20.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 20, 0xff0000);
      }
    }
    
    disableCore0WDT();
  }
}


void setup(){
  M5.begin(true, false, true); //SerialEnable, bool I2CEnable, DisplayEnable

  delay(800);

  Serial1.begin(1000000, SERIAL_8N1, 26, 32); //RX, TX
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(0);
  dxl.torqueOff(1);
  dxl.setOperatingMode(0, OP_CURRENT);
  dxl.setOperatingMode(1, OP_CURRENT);
  dxl.torqueOn(0);
  dxl.torqueOn(1);
  
  delay(400);

  preferences.begin("parameter",false);
  Kp = preferences.getFloat("Kp",Kp);
  Kd = preferences.getFloat("Kd",Kd);
  Kw = preferences.getFloat("Kw",Kw);
  Ki = preferences.getFloat("Ki",Ki);
  Kyaw = preferences.getFloat("Kyaw",Kyaw);

  Kvel = preferences.getFloat("Kvel",Kvel);
  Kroll = preferences.getFloat("Kroll",Kroll);
  
  Delay = preferences.getInt("Delay",Delay);
  offset = preferences.getFloat("offset",offset);

  M5.IMU.Init();

  //IMU フルスケールレンジ
  M5.IMU.SetAccelFsr(M5.IMU.AFS_2G);
  M5.IMU.SetGyroFsr(M5.IMU.GFS_250DPS);

  get_theta();
  kalman.setAngle(theta);


  // Core0 start
  xTaskCreatePinnedToCore(core0,"core0",4096,NULL,1,NULL,0);

  delay(500);
}


void loop(){
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  dt = (float)loopTime / 1000000.0; //sec

  get_theta();
  get_gyro();
    
  kalAngle = kalman.getAngle(theta, gyroX, dt);
  kalAngleDot  = kalman.getRate();


  if(fabs(kalAngle) < 2.0 && GetUP == 0) GetUP = 80;

  if(GetUP==80){
    //ブレーキ
    if(fabs(kalAngle) > 40.0){
      M0 = 0.0;
      M1 = 0.0;
      theta0_hat = 0.0;
      GetUP = 0;
    }

    //モータ回転
    float v0, v1;
    v0 = dxl.getPresentVelocity(0, UNIT_PERCENT);
    v1 = dxl.getPresentVelocity(1, UNIT_PERCENT);
    
    vel = (v0 - v1) * 0.5;   // 左右平均回転速度
    theta0_hat += Ki * (vel - velCmdFiler) * dt;
    theta0_hat *= 0.999;
    
    
    velCmdFiler += 0.1 * (velCmd - velCmdFiler);
    
    
    Mbal = Kp * sin((kalAngle + theta0_hat) * DEG_TO_RAD)
        + Kd / 2500.0 * kalAngleDot
        + Kw / 100.0 * (vel - velCmdFiler);

    Tturn = Kyaw / 100.0 * (gyroZ - omegaCmd);
    M0 = Mbal - Tturn;
    M1 = Mbal + Tturn;
        
  }else if(GetUP==0){
    M0 = 0.0;
    M1 = 0.0;
  }
  
  M0 = 100.0 * constrain(M0, -1.0, 1.0);
  M1 = 100.0 * constrain(M1, -1.0, 1.0);

  dxl.setGoalCurrent(0, M0, UNIT_PERCENT);
  dxl.setGoalCurrent(1, -M1, UNIT_PERCENT);

  delay(Delay);
}
