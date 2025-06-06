#include <Arduino.h>
#include <Wire.h>
#include "driver/i2s.h"
#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_STDIO
#include "minimp3.h"

#include "SD.h"
#include "SPI.h"

#define RDAW 0x11  // 0b0010000
#define SUP 4    // Channel Up
#define PLAY 17
#define RadioOff 16

#define BUFFER_SIZE 1024
#define MENUROW_BUFSIZE 100
#define MENUCOL_BUFSIZE 100

#define I2S_LRC  14
#define I2S_DOUT 26
#define I2S_BCLK 27

volatile bool stopPlayback = false;
bool radio_I2S = true;
bool radio_off = false;
TaskHandle_t playTaskHandle = NULL;
bool is_i2s_configured = false;
SemaphoreHandle_t i2sMutex = NULL;


int SCREEN_ONE_MENU_ROW = 0;
char menuBuffer[MENUROW_BUFSIZE][MENUCOL_BUFSIZE];

bool checkIfitisDirectory(fs::FS &fs,const char *dirName, uint8_t level){
  File root = fs.open(dirName);
  if(!root){
    Serial.printf("\nFile not opened %s \n",dirName);
    return false;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    root.close();
    return false;
  } else {
    root.close();
    return true;
  }

}

void listDir(fs::FS &fs, const char *dirName, uint8_t level,
  char buffer[MENUROW_BUFSIZE][MENUCOL_BUFSIZE],
  int *MENU_ROW)
{
    Serial.printf("Name: %s \n",dirName);

  if(strcmp(dirName,"/FOUND.000") == 0)
  {
      Serial.printf("Inside cmp: %s \n",dirName);

    return;
  }
  Serial.printf("\nListing directory: %s\n,NOW ROW: %d \n",dirName,MENU_ROW);
  File root = fs.open(dirName);
  size_t pathLen;
  if(!root)
  {
    Serial.println("Fialed to open the directory");
    return;
  }
  if(!root.isDirectory())
  {
    Serial.println("Not a Directory");
    root.close();
    return;
  }

  File file = root.openNextFile();

  while(file)
  {
    if(*MENU_ROW >= MENUROW_BUFSIZE)
    {
      Serial.println("BUFFER IS FULL");
      break;
    }
    if(file.isDirectory()){
      if(strcmp(file.path(), "/FOUND.000") == 0) {
      file = root.openNextFile();
      continue;
      }
      Serial.print("File: ");
      Serial.print(file.name());
      Serial.print(" Path: ");
      Serial.println(file.path());
      pathLen = strlen(file.path());
      
      if(pathLen < MENUCOL_BUFSIZE)
      {
        strcpy(buffer[*MENU_ROW],file.path());
        (*MENU_ROW) += 1;
      } else {
        Serial.println("Path too ling");
        return;
      }
      
      if(level){
        listDir(fs,file.path(),level-1,buffer,MENU_ROW);
      }
    } else {
      Serial.print("File: ");
      Serial.print(file.name());
      Serial.print(" Path: ");
      Serial.println(file.path());
      pathLen = strlen(file.path());
      if(pathLen < MENUCOL_BUFSIZE)
      {
        strcpy(buffer[*MENU_ROW],file.path());
        (*MENU_ROW) += 1;
      } else {
        Serial.println("Path too ling");
        return;
      }
    }
    file = root.openNextFile();

  }
  root.close();
  Serial.printf("NOW ROW: %d \n",*MENU_ROW);

}





void esp_i2s_off(){

    if(is_i2s_configured == true){

        i2s_driver_uninstall(I2S_NUM_0); //stop & destroy i2s driver
        Serial.println("I2S uninstalled");
        gpio_reset_pin((gpio_num_t)I2S_BCLK);
        gpio_reset_pin((gpio_num_t)I2S_LRC);
        gpio_reset_pin((gpio_num_t)I2S_DOUT);
        is_i2s_configured = false;
    }
  

}



void i2s_setUp(uint32_t sample_rate){

        if(is_i2s_configured) {
        esp_i2s_off();
        }
  static const i2s_config_t i2s_config = {
            .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
            .sample_rate = (uint32_t)sample_rate,
            .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
            .channel_format = (i2s_channel_fmt_t)I2S_CHANNEL_MONO,
            .communication_format = I2S_COMM_FORMAT_STAND_MSB,
            .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
            .dma_buf_count = 10,
            .dma_buf_len = 512,
            .use_apll = false,
            .tx_desc_auto_clear = true

};

i2s_pin_config_t pin_config = {
  .bck_io_num = I2S_BCLK,
  .ws_io_num  = I2S_LRC,
  .data_out_num = I2S_DOUT,
  .data_in_num = I2S_PIN_NO_CHANGE
};


  i2s_driver_install(I2S_NUM_0,&i2s_config,0,NULL);
  i2s_set_pin(I2S_NUM_0,&pin_config);
  i2s_set_clk(I2S_NUM_0,sample_rate,I2S_BITS_PER_SAMPLE_16BIT,I2S_CHANNEL_MONO);
  is_i2s_configured = true;


}


void play_task(void *parameter) {

        int mpUp = 0;
        char buffer[10][40] = {"/Raabta Kehte Hain Khuda Ne.mp3","/MemoryReboot.mp3","/INS.mp3","/MKV.mp3","/zara.mp3","/PKV.mp3"};
        bool stop = false;
        bool changed = false;
        
        File file = SD.open(buffer[mpUp], "r");
              
        // Init decoder, buffers (simplified for clarity)
        mp3dec_t mp3d;
        mp3dec_init(&mp3d);
        uint8_t input_buf[BUFFER_SIZE];
        short pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
        mp3dec_frame_info_t info = {};
        int buffered = 0;
        float volumeMultiplier = 0.45f; 

 
         while(!stopPlayback) {

                    Serial.println("Inside Play Task");
                    
                    if (!file) {
                      if(changed == false){
                        mpUp += 1;
                        if(mpUp > 5) {mpUp = 0;}

                      }
                      changed = false;
                        

                        file = SD.open(buffer[mpUp], "r");
                        if(!file){
                            Serial.println("Failed to open MP3 file");
                            continue;

                        }

                    }
                    while (file.available()) {
                      
                        size_t n = file.read(input_buf + buffered, BUFFER_SIZE - buffered);
                        buffered += n;

                        int samples = mp3dec_decode_frame(&mp3d, input_buf, buffered, pcm, &info);
                        buffered -= info.frame_bytes;
                        memmove(input_buf, input_buf + info.frame_bytes, buffered);

                        if (digitalRead(PLAY) == HIGH) {
                                    delay(150);
                        if(digitalRead(PLAY) == HIGH){
                            mpUp+=1;
                            Serial.println(mpUp);
                            changed = true;
                            if(mpUp > 5){
                                mpUp = 0;
                            }
                            file.close();               
                            mp3dec_init(&mp3d);

                            buffered = 0;
                            memset(input_buf,0,sizeof(input_buf));
                            memset(pcm,0,sizeof(pcm));
                            break;
                            
                        }
                    }
                        if (samples > 0) {
                            if (!is_i2s_configured) {
                                i2s_setUp(info.hz);
                            }

                            // Downmix stereo to mono if needed
                            if (info.channels == 2) {
                                for (int i = 0; i < samples; i++) {
                                    pcm[i] = (pcm[i * 2] + pcm[i * 2 + 1]) / 2;
                                    pcm[i] = (short)((float)pcm[i] * volumeMultiplier);

                                }
                            } else { // For mono keep it normal
                                for (int i = 0; i < samples; i++) {
                                      pcm[i] = (short)((float)pcm[i] * volumeMultiplier);

                                  }
                            }

                            size_t bytes_written;
                            i2s_write(I2S_NUM_0, pcm, samples * sizeof(short), &bytes_written, portMAX_DELAY);
                        }
                    }

                    file.close();
                    Serial.println("Playback changed or stopped");

                }

                if(stopPlayback == true){
                  if(file) file.close();
                  esp_i2s_off();
                  vTaskDelay(10 / portTICK_PERIOD_MS); // Give it time to exit cleanly
                  playTaskHandle = NULL;
                  vTaskDelete(NULL);
                }
                  
        
   
}

void readRegister(uint8_t reg) {
  Wire.beginTransmission(0x11);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x11,2,1);
  if(Wire.available() >= 2){
     uint16_t value = Wire.read() << 8 | Wire.read();
  Serial.println(value,HEX);
  }
 
}


void writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(RDAW);
  Wire.write(reg);
  Wire.write(highByte(value));
  Wire.write(lowByte(value));
  Wire.endTransmission();
}

void setup() {
  Wire.begin(21, 22);  // SDA, SCL (for ESP32)
  pinMode(SUP,INPUT_PULLDOWN);
  pinMode(PLAY,INPUT_PULLDOWN);
  pinMode(RadioOff,INPUT_PULLDOWN);

  Serial.begin(115200);


  if(!SD.begin()){
    Serial.println("SD Card mounting failed");
    return;
  }

  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE)
  {
    Serial.println("No SD Card is attached");
    return;
  } else if(cardType == CARD_MMC)  {
    Serial.println("MMC");
  } else if( cardType == CARD_SD){
    Serial.println("SDSC");
  } else if( cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize); 
  listDir(SD,"/", 0,menuBuffer,&SCREEN_ONE_MENU_ROW);
  Serial.println("####################################");

  for(int l = 0; l < SCREEN_ONE_MENU_ROW; l++)
  {
    Serial.println(menuBuffer[l]);
  }

  Serial.println("####################################");

  writeRegister(0x02,0b1111000000001001); //  1111000000001001 0xF009
  delay(100);
  writeRegister(0x05,0b1000100010001111); //  1000100010001111 0x888F
  delay(100);  // Wait for tuning 
  writeRegister(0x03,0x490);  
  delay(100);
  writeRegister(0x04,0b0000001001000000); //0000001001000000 = 0x240
  delay(100);
  writeRegister(0x06,0b0000101010000000); //0b0000101010000000 = 0xA80
  delay(100);
  Serial.println("RDA5807FP initialized.");

  //i2sMutex = xSemaphoreCreateMutex();
  //xTaskCreatePinnedToCore(play_task,"MP3 PLAYER",32768,NULL,1,&playTaskHandle,0);

}

/**
 * 0x890 = 90.4Mhz
 * 0x490 = 88.8Mhz
 * 0x1290 = 94.4Mhz
 * 0x2810 = 103.0 Mhz
 */

uint16_t ch[4] = {0x490,0x890,0x1290,0x2810};
int chS = 0;    // Channel Select
void loop() {
  // Main loop
    if(digitalRead(SUP) == HIGH){
        delay(50);
        if(digitalRead(SUP) == HIGH){
            chS += 1;
            if(chS > 3){chS = 0;}
            Serial.println(chS);
            writeRegister(0x03,ch[chS]);
            delay(100);
        }
         
    }
     if(digitalRead(RadioOff) == HIGH){
        delay(50);
        if(digitalRead(RadioOff) == HIGH){  
          if(radio_off == false){
            writeRegister(0x02,0); 
            delay(100);
            writeRegister(0x04,0);   
            delay(100);
            writeRegister(0x03,0);      
            delay(100); 
            writeRegister(0x05,0);
            delay(100);
            stopPlayback = false; // Reset stop flag
            xTaskCreatePinnedToCore(play_task,"MP3 PLAYER",32768,NULL,1,&playTaskHandle,1);

            Serial.println("Radio Off");
            radio_off = true;
          } else if(radio_off == true){
            stopPlayback = true; // Reset stop flag
            //esp_i2s_off();
           // vTaskDelay(100 / portTICK_PERIOD_MS); // Give it time to exit cleanly
           uint32_t timeout = 200;
           while (playTaskHandle != NULL && timeout-- < 200);{
            vTaskDelay(1/portTICK_PERIOD_MS);
           }
           
            if(playTaskHandle != NULL){
              if(is_i2s_configured == true){
                esp_i2s_off();
              }
            vTaskDelete(playTaskHandle);

            }

            writeRegister(0x02,0b1111000000001001); // Enable 
            delay(100);
            writeRegister(0x05,0b1000100010001111); //  1000100010001111 0x888F
            delay(100);  // Wait for tuning
            writeRegister(0x04,0b0000001001000000); //0000001001000000 = 0x240
            delay(100);
            writeRegister(0x06,0b0000101010000000); //0b0000101010000000 = 0xA80
            delay(100);
            writeRegister(0x03,ch[chS]);  // 000000100010010000 = 0x890 // 90.4Mhz
            delay(100);
            Serial.println(chS);
            Serial.println("Radio On");
            radio_off = false;
          }
          delay(100);
        }
         
    }


}



 