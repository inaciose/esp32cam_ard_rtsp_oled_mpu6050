// ################################################################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ################################################################

// if give an error about BUFFER_LENGTH on compilation
// on library file MPU6050.cpp, insert the following line
// #define BUFFER_LENGTH 32
// at the top or just before the following line (near 2751)
//int8_t MPU6050::GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof

// ################################################################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ###########     ATENTION - ATENTION     ########################
// ################################################################

#include "OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include "SimStreamer.h"
#include "OV2640Streamer.h"
#include "CRtspSession.h"

// for led blink
//#define LED_BUILTIN 33
#ifdef LED_BUILTIN
bool ledon = false;
#endif

//#define ENABLE_MPU6050 //if want use oled,turn on this macro
#define ENABLE_OLED //if want use oled,turn on this macro
// #define SOFTAP_MODE // If you want to run our own softap turn this on
#define ENABLE_WEBSERVER
#define ENABLE_RTSPSERVER

#if defined(ENABLE_OLED) || defined(ENABLE_MPU6050)
#include <Wire.h>
// The pins for I2C are defined by the Wire-library, or in this case,
// in the following lines

// -----------------I2C-----------------
// -- NEED TO DEFINE PINS IN ESP32CAM --
// ----------- 3 NEW LINES -------------
#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15
#endif

#ifdef ENABLE_MPU6050
// mpu6050
#define M_PI 3.14159265358979323846
#define __PGMSPACE_H_ 1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define DMP_INTERRUPT_PIN GPIO_NUM_13

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
#endif

#ifdef ENABLE_OLED
// oled
#include "SSD1306.h"
#define OLED_ADDRESS 0x3c
#define I2C_SDA 14
#define I2C_SCL 15
SSD1306Wire display(OLED_ADDRESS, I2C_SDA, I2C_SCL, GEOMETRY_128_64);
bool hasDisplay; // we probe for the device at runtime
#endif

OV2640 cam;

#ifdef ENABLE_WEBSERVER
WebServer server(80);
#endif

#ifdef ENABLE_RTSPSERVER
WiFiServer rtspServer(8554);
#endif

#ifdef SOFTAP_MODE
IPAddress apIP = IPAddress(192, 168, 1, 1);
#else
#include "wifikeys.h"
#endif

#ifdef ENABLE_MPU6050
// mpu6050
// dmp isr
static void IRAM_ATTR dmpDataReady(void * arg) {
    mpuInterrupt = true;
}
#endif

#ifdef ENABLE_MPU6050
void mpuDMP() {
    static uint32_t timer = millis();

    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        if(millis() > timer) {
            timer = millis()+50;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
        }
    }
}
#endif


#ifdef ENABLE_WEBSERVER
void handle_jpg_stream(void)
{
    WiFiClient client = server.client();
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    server.sendContent(response);

    while (1)
    {
        cam.run();
        if (!client.connected())
            break;
        response = "--frame\r\n";
        response += "Content-Type: image/jpeg\r\n\r\n";
        server.sendContent(response);

        client.write((char *)cam.getfb(), cam.getSize());
        server.sendContent("\r\n");
        if (!client.connected())
            break;
    #ifdef ENABLE_MPU6050
        mpuDMP();
    #endif
    }
}

void handle_jpg(void)
{
    WiFiClient client = server.client();

    cam.run();
    if (!client.connected())
    {
        return;
    }
    String response = "HTTP/1.1 200 OK\r\n";
    response += "Content-disposition: inline; filename=capture.jpg\r\n";
    response += "Content-type: image/jpeg\r\n\r\n";
    server.sendContent(response);
    client.write((char *)cam.getfb(), cam.getSize());
}

void handleNotFound()
{
    String message = "Server is running!\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    server.send(200, "text/plain", message);
}
#endif

void lcdMessage(String msg)
{
  #ifdef ENABLE_OLED
    if(hasDisplay) {
        display.clear();
        display.drawString(128 / 2, 32 / 2, msg);
        display.display();
    }
  #endif
}

void setup()
{
    
    // INIT SERIAL (debug purposes)
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    Serial.println(F("Serial connected"));

    //Wire.begin(I2C_SDA, I2C_SCL, 4000000);

  #ifdef ENABLE_OLED
    
    hasDisplay = display.init();
    if(hasDisplay) {
        display.flipScreenVertically();
        display.setFont(ArialMT_Plain_16);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
    }
    
  #endif
    
#ifdef LED_BUILTIN
    // led blink
    pinMode(LED_BUILTIN, OUTPUT);
#endif

    lcdMessage("booting");

    //cam.init(esp32cam_config);
    cam.init(esp32cam_aithinker_config);

    IPAddress ip;

#ifdef SOFTAP_MODE
    const char *hostname = "devcam";
    // WiFi.hostname(hostname); // FIXME - find out why undefined
    lcdMessage("starting softAP");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    bool result = WiFi.softAP(hostname, "12345678", 1, 0);
    if (!result)
    {
        Serial.println("AP Config failed.");
        return;
    }
    else
    {
        Serial.println("AP Config Success.");
        Serial.print("AP MAC: ");
        Serial.println(WiFi.softAPmacAddress());

        ip = WiFi.softAPIP();
    }
#else
    lcdMessage(String("join ") + ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(F("."));
    }
    ip = WiFi.localIP();
    Serial.println(F("WiFi connected"));
    Serial.println("");
    Serial.println(ip);
#endif

    lcdMessage(ip.toString());

#ifdef ENABLE_WEBSERVER
    server.on("/", HTTP_GET, handle_jpg_stream);
    server.on("/jpg", HTTP_GET, handle_jpg);
    server.onNotFound(handleNotFound);
    server.begin();
#endif

#ifdef ENABLE_RTSPSERVER
    rtspServer.begin();
#endif

#ifdef ENABLE_MPU6050
    // mpu6050
    // initialize device
    Serial.println(F("First MPU6050 initialization ..."));
    mpu.initialize();
    delay(100);

    Serial.println(F("MPU6050 reset..."));
    mpu.reset(); //help startup reliably - doesn't always work though.
    // maybe can also reset i2c on esp32?
    delay(100);

    Serial.println(F("MPU6050 resetI2CMaster..."));
    mpu.resetI2CMaster();
    delay(100);

    // initialize device again
    Serial.println(F("Final MPU6050 initialization..."));
    mpu.initialize();
    
    pinMode(DMP_INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        /*
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (mcu external interrupt "));
        Serial.print(digitalPinToInterrupt(DMP_INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(DMP_INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        */

        esp_err_t err;

        err = gpio_isr_handler_add(DMP_INTERRUPT_PIN, &dmpDataReady, (void *) 13);
        if (err != ESP_OK) {
            Serial.printf("handler add failed with error 0x%x \r\n", err);
        }

        err = gpio_set_intr_type(DMP_INTERRUPT_PIN, GPIO_INTR_POSEDGE);
        if (err != ESP_OK) {
            Serial.printf("set intr type failed with error 0x%x \r\n", err);
        }

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        fifoCount = mpu.getFIFOCount();


    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
#endif
}


CStreamer *streamer;
CRtspSession *session;
WiFiClient client; // FIXME, support multiple clients


void videoLoop()
{
#ifdef ENABLE_WEBSERVER
    server.handleClient();
#endif

#ifdef LED_BUILTIN
    // led blink control
    static uint32_t ledtimer = millis();
    if(millis() > ledtimer) {
        ledtimer = millis()+1000;
        if(ledon) {
            digitalWrite(LED_BUILTIN, LOW);
            ledon = false;
        } else {
            digitalWrite(LED_BUILTIN, HIGH);
            ledon = true;  
        }
    }
#endif

#ifdef ENABLE_RTSPSERVER
    uint32_t msecPerFrame = 100;
    static uint32_t lastimage = millis();

    // If we have an active client connection, just service that until gone
    // (FIXME - support multiple simultaneous clients)
    if(session) {
        session->handleRequests(0); // we don't use a timeout here,
        // instead we send only if we have new enough frames

        uint32_t now = millis();
        if(now > lastimage + msecPerFrame || now < lastimage) { // handle clock rollover
            session->broadcastCurrentFrame(now);
            lastimage = now;

            // check if we are overrunning our max frame rate
            now = millis();
            if(now > lastimage + msecPerFrame)
                printf("warning exceeding max frame rate of %d ms\n", now - lastimage);
        }

        if(session->m_stopped) {
            delete session;
            delete streamer;
            session = NULL;
            streamer = NULL;
        }
    }
    else {
        client = rtspServer.accept();

        if(client) {
            //streamer = new SimStreamer(&client, true);             // our streamer for UDP/TCP based RTP transport
            streamer = new OV2640Streamer(&client, cam);             // our streamer for UDP/TCP based RTP transport

            session = new CRtspSession(&client, streamer); // our threads RTSP session and state
        }
    }
#endif
}


void loop() {
#ifdef ENABLE_MPU6050    
    mpuDMP();
#endif
    videoLoop();
}

/*
void loop() {

    while (fifoCount < packetSize) {
        //insert here your code
        videoLoop();
        fifoCount = mpu.getFIFOCount();
    }

    if (fifoCount == 1024) {    
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));    
    } else {
    
        if (fifoCount % packetSize != 0) {
        
            mpu.resetFIFO();
            
        } else {
    
            while (fifoCount >= packetSize) {
                mpu.getFIFOBytes(fifoBuffer,packetSize);
                fifoCount -= packetSize;
            }    
        
            mpu.dmpGetQuaternion(&q,fifoBuffer);
            mpu.dmpGetGravity(&gravity,&q);
            mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);          
            
            Serial.print("ypr\t");
            Serial.print(ypr[0]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[1]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[2]*180/PI);
            Serial.println();
            
        }
   
    }

}
*/