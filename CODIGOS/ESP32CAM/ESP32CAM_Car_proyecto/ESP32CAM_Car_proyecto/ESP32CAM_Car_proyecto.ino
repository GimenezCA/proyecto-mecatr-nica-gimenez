/*/ formas de configurar la IP
 * 1- el codigo estandar: nos brinda una IP aleatoria cada ves que se apaga el equipo.
 * 2- codigo que uniliza la red de la pc, y establecemos una IP fija (usada en el proyecto):
 * 3- codigo para configurar el esp32ca como punto de acceso (AP) y utilizar la red que la placa brinda  
 * 
 * //codigo abierto
//parte del codigo extraido de: www.youtube.com/c/viralscience  www.viralsciencecreativity.com
//ESP32 Camera 
 #include "esp_camera.h"
#include <WiFi.h>
//#include "HardwareSerial.h" // Necesario para usar una instancia de HardwareSerial
//codigo 1-
//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//
// Adafruit ESP32 Feather

//ATENCION: se usa este para el proyecto. 
  
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_M5STACK_PSRAM
#define CAMERA_MODEL_AI_THINKER


const char* ssid = "Claudio_82";   //Enter SSID WIFI Name   Claudio_82
const char* password = "ESP32-cam";   //Enter WIFI Password  ESP32-cam


#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22


#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif


extern int gpLed =  4; // led esp
extern String WiFiAddr ="";

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();


  pinMode(gpLed, OUTPUT); //Light

  
  digitalWrite(gpLed, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  
  //Tamaño de fotograma desplegable para una velocidad de fotogramas inicial más alta
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);


  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
   Serial.print(".");
  }
  Serial.println("");
 Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  WiFiAddr = WiFi.localIP().toString();
  Serial.println("' to connect");
}

void loop() {
  // put your main code here, to run repeatedly:

}*/

// 2-FUNCIONA. USAR ESTO EN CASO DE QUE 3 NO FUNCIONE
//-----------------------------------------
//codigo para dejar ip fija 
//----------------------------------
#include "esp_camera.h"
#include <WiFi.h>
//#include "HardwareSerial.h" // Necesario para usar una instancia de HardwareSerial

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//
// Adafruit ESP32 Feather

//ATENCION: se usa este para el proyecto, guardado en carpeta proyecto 2025... 
  
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_M5STACK_PSRAM
#define CAMERA_MODEL_AI_THINKER


const char* ssid = "Claudio_82";    //Enter SSID WIFI Name    Claudio_82
const char* password = "ESP32-cam";    //Enter WIFI Password  ESP32-cam

// --- CONFIGURACIÓN DE IP FIJA ---
// IMPORTANTE: Debes ajustar estos valores a la configuración de tu red.
// La IP fija debe estar dentro del rango de tu red, pero fuera del rango DHCP
// para evitar conflictos con otras IPs asignadas automáticamente.
IPAddress local_IP(192, 168, 137, 100); // IP que quieres asignar a tu ESP32-CAM///ip fija...cargo en el buscador
IPAddress gateway(192, 168, 137, 1);    // Puerta de enlace de tu red (normalmente la IP de tu router)// de mi compu en este caso
IPAddress subnet(255, 255, 255, 0);     // Máscara de subred (la más común es 255.255.255.0)
// --- FIN CONFIGURACIÓN DE IP FIJA ---


#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     21
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       19
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM        5
#define Y2_GPIO_NUM        4
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM      32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM      26
#define SIOC_GPIO_NUM      27

#define Y9_GPIO_NUM        35
#define Y8_GPIO_NUM        34
#define Y7_GPIO_NUM        39
#define Y6_GPIO_NUM        36
#define Y5_GPIO_NUM        21
#define Y4_GPIO_NUM        19
#define Y3_GPIO_NUM        18
#define Y2_GPIO_NUM         5
#define VSYNC_GPIO_NUM     25
#define HREF_GPIO_NUM      23
#define PCLK_GPIO_NUM      22

#else
#error "Camera model not selected"
#endif


extern int gpLed =  4; // led esp
extern String WiFiAddr ="";

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();


  pinMode(gpLed, OUTPUT); //Light
  
  digitalWrite(gpLed, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  
  //Tamaño de fotograma desplegable para una velocidad de fotogramas inicial más alta
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);

  // --- APLICAR CONFIGURACIÓN DE IP FIJA ---
  WiFi.config(local_IP, gateway, subnet);
  // --- FIN APLICAR CONFIGURACIÓN DE IP FIJA ---

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  WiFiAddr = WiFi.localIP().toString();
  Serial.println("' to connect");
}

void loop() {
  // put your main code here, to run repeatedly:

} 

// funciona. pero el celu debe estar en la misma red que el esp32cam
// guarda la ip fija asi no debemos abrir el monitor serial cada ves que pasa un tiempo
//-------------------------------------------------------------------------



/*/ codigo 3
//-------------------------------------------------------------
//prueba para punto de acceso
//-------------------------------------------------------------
#include "esp_camera.h"
#include <WiFi.h>
//#include "HardwareSerial.h" // Necesario para usar una instancia de HardwareSerial

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//
// Adafruit ESP32 Feather

//ATENCION: se usa este para el proyecto, guardado en carpeta proyecto 2025... 
  
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_M5STACK_PSRAM
#define CAMERA_MODEL_AI_THINKER

// --- CONFIGURACIÓN PARA MODO PUNTO DE ACCESO (AP) ---
// Define el SSID y la contraseña para la red Wi-Fi que creará tu ESP32-CAM.
// Puedes cambiar estos valores a tu gusto.
const char* ap_ssid = "ESP32-CAM-PROYECTO_25";    // Nombre de la red Wi-Fi (SSID) que creará el ESP32-CAM
const char* ap_password = "Proyecto_25"; // Contraseña para la red Wi-Fi (mínimo 8 caracteres)

// En modo AP, la IP del ESP32-CAM por defecto suele ser 192.168.4.1.
// No es necesario configurar IP fija explícitamente como en modo estación,
// ya que el ESP32 es el que asigna las IPs a los clientes.
// --- FIN CONFIGURACIÓN AP ---


#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     21
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       19
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM        5
#define Y2_GPIO_NUM        4
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM      32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM      26
#define SIOC_GPIO_NUM      27

#define Y9_GPIO_NUM        35
#define Y8_GPIO_NUM        34
#define Y7_GPIO_NUM        39
#define Y6_GPIO_NUM        36
#define Y5_GPIO_NUM        21
#define Y4_GPIO_NUM        19
#define Y3_GPIO_NUM        18
#define Y2_GPIO_NUM         5
#define VSYNC_GPIO_NUM     25
#define HREF_GPIO_NUM      23
#define PCLK_GPIO_NUM      22

#else
#error "Camera model not selected"
#endif


extern int gpLed =  4; // led esp
extern String WiFiAddr =""; // This variable will now store the AP IP address

void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();


  pinMode(gpLed, OUTPUT); //Light
  
  digitalWrite(gpLed, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  
  //Tamaño de fotograma desplegable para una velocidad de fotogramas inicial más alta
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);

  // --- INICIAR MODO PUNTO DE ACCESO (AP) ---
  // Deshabilitar el modo estación (si estuviera habilitado)
  WiFi.mode(WIFI_AP);
  // Iniciar el punto de acceso con el SSID y la contraseña definidos
  // El tercer parámetro (canal) es opcional, 1 por defecto.
  // El cuarto parámetro (oculto) es opcional, false por defecto.
  // El quinto parámetro (máx. conexiones) es opcional, 4 por defecto.
  if (WiFi.softAP(ap_ssid, ap_password)) {
    Serial.print("Punto de Acceso '" + String(ap_ssid) + "' creado con éxito. Contraseña: '" + String(ap_password) + "'");
    Serial.print("IP del AP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("¡Error al crear el Punto de Acceso!");
  }
  // --- FIN INICIAR MODO AP ---

  // Las siguientes líneas son para el modo estación y se comentan/eliminan para el modo AP
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("");
  // Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Conéctate a la red WiFi: '");
  Serial.print(ap_ssid);
  Serial.print("' y luego usa 'http://");
  Serial.print(WiFi.softAPIP()); // Obtener la IP del AP
  WiFiAddr = WiFi.softAPIP().toString();
  Serial.println("' para conectar");
}

void loop() {
  // put your main code here, to run repeatedly:

}

// funciona como punto de acceso, pero no puedo verlo desde el celu, pero si desde la compu. 
// se tilizara 2 para el proyecto.*/
