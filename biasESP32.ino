/*
      **************************************************************************************************************
                   # Bu kod, KahveLAB Elektron Kaynak Makinasi projesinde kullanilan 
                   # Bipolar guc kaynagini (BIAS) kontrol etmek,
                   # BIAS bipolar gerilimini okumak
                   # ve yuksek gerilimdeki guc kaynaklarinin 
                   # calisma durumunu belirten ledleri kontrol etmek icin kullanilan 
                   # ESP32 cipi icin 08/07/2021 tarihinde Muhammet Furkan Er tarafindan yazilmistir.
      **************************************************************************************************************
      
            * ESP32, ADS1115 gerilim olcme modulu ile I2C haberlesmesi,
            * ESP32, DAC8563 gerilim kaynagiyla SPI haberlesmesi,
            * ESP32, 3 adet 2n2222 transistor, 3 adet 1k ohm resistorler yardimi ile 3 adet Siemens J9906 role kontrolu,
            yapmaktadir.

      ** Asagida #STASSSID# adi ile verilen aga #STAPSK# sifresiyle baglanip, 
      ** #staticIP# IP adresinde ve #port# portunda bir server kurup,
      ** Server kurdugunu ve baglanti bekledigini HV Rackte bulunun 3 ledi sirayla acip kapatarak belirtir.
      ** Baglanti kuramadigi durumlarda ise, 3 ledi ayni anda acip kapatarak,
      ** Verilen #STASSSID# ve #STAPSK# ile bir baglanti kuramadigini ve aramaya devam ettigini belirtir.


      ** Kurdugu servera katilan clientlara ADS1115 den aldigi veriyi #handleOutgoing# fonksiyonu ile gonderir.
      ** Kurdugu servera katilan clientlarin gonderdigi verileri #handleIncoming# fonksiyonu ile dinler,
      ** ve gerekli formatta gonderilmis veriyi #DAC8563# e ve rolelere gonderir.
     
     
      ** ADS1115 devre uzerinden beslendigi 3.3V degerinin uzerinde bir analong giris kabul edemedigi icin maksimum ve minimum okuma degerleri BIAS icin +-2000V.
      ** Bu sebepten dolayi, #DAC8563# e 22315-43220 (uint16_t) araliginda bir deger girilmesi gerekmektedir. Bu degerler #DAC8563# 'un -+3.3V ve BIAS'in -+2000V vermesi demektir.
*/



#include <ADS1115_lite.h>
#include <SPI.h>
#include <string.h>

using std::string;


#include <WiFi.h>

#ifndef STASSID
#define STASSID "ssid"                                       //Kurulan wifi'in SSID
#define STAPSK  "pass"                           //Kurulan wifi'in sifresi
#endif

#define BAUD_SERIAL 115200                                  //Serial debug icin istenilen Baud Rate
#define STACK_PROTECTOR  512 
#define MAX_SRV_CLIENTS 2                                   //Maksimum client sayisi


IPAddress staticIP(192,168,137,76);                           //Istenilen statik ip adresi
IPAddress gateway(192,168,137,5);                             //Modem ip adresi
IPAddress subnet(255,255,255,0);
const char* ssid = STASSID;
const char* password = STAPSK;
const int port = 23;                                        //Acilan serverin portu


WiFiServer server(port);                                  
WiFiClient serverClients[MAX_SRV_CLIENTS];


#define R_1 17                                                //Roleleri kontrol eden pin numaralari       
#define R_2 18
#define R_3 19

const int role[3]= {R_1, R_2, R_3};                  
const long int relay[3]={1,2,4};    

#define HSPI_MISO   12                                              //Spipinleri ve clock frekansi ayari
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define HSPI_SS     26
static const int spiClk = 1000000; // SET SPI FREQ to 1 MHz





float a = 0;
int relayStatus=0;
uint16_t dacValue=32768;
int count=0;

SPIClass * hspi = NULL;
ADS1115_lite ads(ADS1115_ADDRESS_ADDR_GND);
















void ads_config(){                                                                    //ADC ayari
  ads.setGain(ADS1115_REG_CONFIG_PGA_4_096V);                                         // max 4.096    (2^16 - 1)=4.096V
  ads.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS);                                    //Ortalama hiz - cozunurluk kaybi yok
}

int16_t ads_read1(){                                                                  //AIN1 pini okuma fonksiyonu
  ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);                                        //Single mode input on AIN1
  ads.triggerConversion();  
  return ads.getConversion();  
}

int16_t ads_read2(){                                                                   //AIN2 pini okuma fonksiyonu
  ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);                                         //Single mode input on AIN2
  ads.triggerConversion();  
  return ads.getConversion();  
}

String ReadBias(){                                                              //Bias cikisini okuyup n04568 seklinde bir string veren kullanimdan kalkmis eski fonksiyon.
    String biasV;   
    int16_t b_p;
    int16_t b_n;                                    
    b_p = ads_read1();                              
    b_n = ads_read2();                            
    if(b_p >= b_n){                           
      Serial.println(((float)b_p - 160)/8000);
      biasV = String("p");
      if(b_p < 10000){
        biasV += "0";
        if(b_p < 1000){
          biasV += "0";
          if(b_p < 100){
            biasV += "0";
            if(b_p < 10){
              biasV += "0";
            }
          }
        }    
      }
      biasV += String(b_p);
    }
    else{
      Serial.println(((float)b_n)*0.91/8000);
      biasV = String("n");
      if(b_n < 10000){
        biasV += "0";
        if(b_n < 1000){
          biasV += "0";
          if(b_n < 100){
            biasV += "0";
            if(b_n < 10){
              biasV += "0";
            }
          }
        }                                    
      }
      biasV += String(b_n);
    }       
    return biasV;                                         
}



int16_t biasAdc(){                                                                                  //Bias cikisini okuyup uint16_t bit seklinde donduren fonksiyon
  int16_t b_p = ads_read1();
  int16_t b_n = ads_read2();
  return (b_p > b_n) ? b_p : (0x8000 - b_n) | 0x8000;
}




void writeToA(SPIClass *spi,int sspin,uint16_t value){                                              //DAC8563 icin A portuna uint16_t yazma fonksiyonu. UINT16_MAX/2 ustu pozitif, alti negatif.
  byte dat[3];                                                                                      //Formul -- f(x)=(UINT16_MAX/2 + x) --         f(-UINT16_MAX/2) = -10.45V , f(UINT16_MAX/2) = 10.45V
  dat[0] = 0b00011000;
  dat[1] = (value >> 8) & 0xFF;
  dat[2] = value & 0xFF;
  digitalWrite(sspin, LOW);
  spi->transfer(dat[0]);
  delay(1);
  spi->transfer(dat[1]);
  delay(1);
  spi->transfer(dat[2]);
  digitalWrite(sspin, HIGH);
}

uint16_t voltToUINT16(float value){                                                                 //Yukaridaki formule gore verilen voltu uint16_t seklinde gonderilmeye hazir 2 byte veriye ceviren fonksiyon
  return (10.45 + value)*65535/20.8;
}

void resetReg(SPIClass *spi,int sspin){                                                             //Reset register of DAC
  byte dat[3];
  dat[0] = 0b00101000;
  dat[1] = 0b00000000;
  dat[2] = 0b00000001;
  //spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(sspin, LOW);
  spi->transfer(dat[0]);
  delay(1);
  spi->transfer(dat[1]);
  delay(1);
  spi->transfer(dat[2]);
  digitalWrite(sspin, HIGH);
  //spi->endTransaction();
}


void powerUpA(SPIClass *spi,int sspin){                                                             //Dac A portuna guc ver
  byte dat[3];
  dat[0] = 0b00100000;
  dat[1] = 0b00000000;
  dat[2] = 0b00000011;
  //spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(sspin, LOW);
  spi->transfer(dat[0]);
  delay(1);
  spi->transfer(dat[1]);
  delay(1);
  spi->transfer(dat[2]);
  digitalWrite(sspin, HIGH);
  //spi->endTransaction();
}


void enableIntRef(SPIClass *spi,int sspin){                                                         //İc referansi ac. Daha fazla bilgi icin DAC8563 doc a bak.
  byte dat[3];
  dat[0] = 0b00111000;
  dat[1] = 0b00000000;
  dat[2] = 0b00000001;
  //spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(sspin, LOW);
  spi->transfer(dat[0]);
  delay(1);
  spi->transfer(dat[1]);
  delay(1);
  spi->transfer(dat[2]);
  digitalWrite(sspin, HIGH);
  //spi->endTransaction();
}


void gainSet(SPIClass *spi,int sspin){                                                                //A portu icin gain set et.
  byte dat[3];
  dat[0] = 0b00000010;
  dat[1] = 0b00000000;
  dat[2] = 0b00000000;
  //spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(sspin, LOW);
  spi->transfer(dat[0]);
  delay(1);
  spi->transfer(dat[1]);
  delay(1);
  spi->transfer(dat[2]);
  digitalWrite(sspin, HIGH);
  //spi->endTransaction();
}

void ldacDisable(SPIClass *spi,int sspin){                                                              //LDAC inaktif.
  byte dat[3];
  dat[0] = 0b00110000;
  dat[1] = 0b00000000;
  dat[2] = 0b00000011;
  //spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  digitalWrite(sspin, LOW);
  spi->transfer(dat[0]);
  delay(1);
  spi->transfer(dat[1]);
  delay(1);
  spi->transfer(dat[2]);
  digitalWrite(sspin, HIGH);
  //spi->endTransaction();
}




void handleOutgoing(int16_t adc,WiFiClient *serverClients){                             //ESPden giden verileri handle fonksiyonu.
   for (int i = 0; i < MAX_SRV_CLIENTS; i++){
    if(serverClients[i]){                                                               //Her clienta biasdan okudugu 2 bytelik veriyi basinda 'b' headeri ve sonunda '*' olacak sekilde gonderiyor.
      serverClients[i].write("b");
      serverClients[i].write((adc >> 8) & 0xFF);
      serverClients[i].write(adc & 0xFF);
      serverClients[i].write("*");    
    }
  }
  Serial.println(adc);
}




void handleIncoming(int *rS, uint16_t *dac,WiFiClient *serverClients){                  //Gelen mesajlari handle etme fonksiyonu.
  String cmd;                                                                           //rS integerina role verilerini bitler olarak, dac'a ise DAC8563'e yazilmak istenen 2 bytelik veriyi kaydediyor.   
  for (int i = 0; i < MAX_SRV_CLIENTS; i++){                                               
    while (serverClients[i].available() > 0) {
      size_t maxToSerial = serverClients[i].available();
      maxToSerial = std::min(maxToSerial, (size_t)STACK_PROTECTOR);
      uint8_t buf[maxToSerial];
      size_t tcp_got = serverClients[i].read(buf, maxToSerial);
      char cm[tcp_got];
      for(int j=0 ; j < tcp_got ; j++){
        if(buf[j] == '*'){
          cm[j]='\0';
          break;
        }
        cm[j]=buf[j];
       }
       charToString(cm,cmd);
       cmd.remove(0,cmd.indexOf("dac")+3);                                              //Gelen mesaj "dac#DACVALUE#re#relayValue#*" seklinde.
       *dac = cmd.substring(0,cmd.indexOf("re")).toInt();
       cmd.remove(0,cmd.indexOf("re")+2);
       *rS = cmd.toInt();
       serverClients[i].flush();                                                        //Fonksiyon biterken clientlarn gonderdigi sonraki verileri RAM de yer acmak icin siliyor.
      }
   }
}











void setup() {
  pinMode(R_1, OUTPUT);
  pinMode(R_2, OUTPUT);
  pinMode(R_3, OUTPUT);
                                                    
  Serial.begin(BAUD_SERIAL);
  WiFi.mode(WIFI_STA);      
  Serial.print("status:\n");
  Serial.println(WiFi.status());
  WiFi.config(staticIP, gateway, subnet);                                                   //Wifi ayarlari yapildi
  WiFi.begin(ssid, password);                                                               //Wifi'a baglaniyor.
  while (WiFi.status() != 3) {                                                              //Baglanana kadar deniyor.
    WiFi.begin(ssid,password);
    digitalWrite(R_1,HIGH);  
    digitalWrite(R_2,HIGH);  
    digitalWrite(R_3,HIGH);
    delay(2500);
    digitalWrite(R_1,LOW);  
    digitalWrite(R_2,LOW);  
    digitalWrite(R_3,LOW);  
    delay(2500);
    Serial.println(WiFi.status());
  }
  server.begin();
  server.setNoDelay(true);
  Serial.println("Server created. Use the ip and port:*");
  Serial.println(WiFi.localIP());
  Serial.println(port);
  Serial.println("Ready*");
  delay(10);

  
  pinMode(HSPI_SS, OUTPUT);

  hspi = new SPIClass(HSPI);                                              //DAC Ayarlari yapiliyor.
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);
  digitalWrite(HSPI_SS, HIGH);
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  resetReg(hspi, HSPI_SS);    
  delay(50);
  powerUpA(hspi, HSPI_SS); 
  delay(50);
  enableIntRef(hspi, HSPI_SS); 
  delay(50);
  gainSet(hspi, HSPI_SS); 
  delay(50);
  ldacDisable(hspi, HSPI_SS); 
  
  ads_config();                                                            //ADS Ayarlari yapiliyor.



  digitalWrite(R_1,HIGH);                                                  //Roleler ile baglandigini belirtiyor.
  delay(500);
  digitalWrite(R_2,HIGH);
  delay(500);
  digitalWrite(R_3,HIGH);
  delay(500);
  digitalWrite(R_1,LOW);
  delay(500);
  digitalWrite(R_2,LOW);
  delay(500);
  digitalWrite(R_3,LOW);
  delay(500);
}











void loop() {
  if(WiFi.status() != 3){
    while (WiFi.status() != 3) {                                                              //Baglanti koptugunda roleleri acip kapatiyor ve baglanana kadar deniyor
      WiFi.begin(ssid,password);
      digitalWrite(R_1,HIGH);  
      digitalWrite(R_2,HIGH);  
      digitalWrite(R_3,HIGH);
      delay(2500);
      digitalWrite(R_1,LOW);  
      digitalWrite(R_2,LOW);  
      digitalWrite(R_3,LOW);  
      delay(2500);
      Serial.println(WiFi.status());
    }
    server.begin();
    server.setNoDelay(true);
    digitalWrite(R_1,HIGH);                                                                   //Roleler ile baglandigini belirtiyor.
    delay(500);
    digitalWrite(R_2,HIGH);
    delay(500);
    digitalWrite(R_3,HIGH);
    delay(500);
    digitalWrite(R_1,LOW);
    delay(500);
    digitalWrite(R_2,LOW);
    delay(500);
    digitalWrite(R_3,LOW);
    delay(500);
  }
  
  count++;
  if (server.hasClient()) {                                                                   //Server'a biri baglandiginda buraya giriyor.
    Serial.println("Server has a client");
    int i = 0;
    for (;i < MAX_SRV_CLIENTS; i++)                                                           //Max baglanti sayisina ulasilmadigindan emin olup baglanan clienta isim veriyor.
      if (!serverClients[i]) {
        serverClients[i] = server.available();
        break;
      }
    if (i == MAX_SRV_CLIENTS) {
      server.available().println("busy");
    }
  }
  
  if(count==20){                                                                              //20 loopta bir (yaklasik 1-1.5 saniye) clientlara adcden okudugu veriyi gonderiyor.
     handleOutgoing(biasAdc(),serverClients);    
     count=0;    
  }

  handleIncoming(&relayStatus,&dacValue,serverClients);
  
  writeToA(hspi,HSPI_SS,dacValue);
  
  for(int i=0; i<3; i++){                                                                      //Gelen komut ile bit karsilastirmasi yapip roleleri gerekli konuma getiriyor.
    int stat = ((relayStatus & relay[i]) == relay[i]) ? HIGH:LOW;
    digitalWrite(role[i],stat);
  }
  delay(50);
}


void charToString(char S[], String &D){
 String rc(S);
 D = rc;
}
