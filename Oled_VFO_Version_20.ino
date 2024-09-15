/*
Version 19 : Version 18 nettoyée
*/

#define VERSION  "2024.20"
#include <Wire.h>
#include <OLED_I2C.h>
#define ANALOG_TUNING (A3)   // Entrée analogique A3 de l'arduino comme mesure du pot 10 tours
#define BB0(x) ((uint8_t)x)             // Bust int32 into Bytes
#define BB1(x) ((uint8_t)(x>>8))
#define BB2(x) ((uint8_t)(x>>16))

#define SI5351BX_ADDR 0x60              // I2C address of Si5351   (typical)
#define SI5351BX_XTALPF 3               // 1:6pf  2:8pf  3:10pf

#define SI5351BX_XTAL 25000000          // Crystal freq in Hz
#define SI5351BX_MSA  35                // VCOA is at 25MHz*35 = 875MHz

// User program may have reason to poke new values into these 3 RAM variables
uint32_t si5351bx_vcoa = (SI5351BX_XTAL*SI5351BX_MSA);  // 25mhzXtal calibrate default
uint8_t  si5351bx_rdiv = 0;             // 0-7, CLK pin sees fout/(2**rdiv)
uint8_t  si5351bx_drive[3] = {1, 1, 1}; // 0=2ma 1=4ma 2=6ma 3=8ma for CLK 0,1,2
uint8_t  si5351bx_clken = 0xFF;         // Private, all CLK output drivers off
int fld=7030;
int s=0;
const int nombreech=12;
int ech[nombreech];
int indice=0;
int total=0;
int moyenne=0;

OLED  myOLED(SDA, SCL); // Remember to add the RESET pin if your display requires it...
extern uint8_t BigNumbers[]; // lié à la librairie OLED_I2C

// modify this section to affect outputs of Si5351
#define OutBFO  2 //clk 2 X3 output
#define OutLO2  1 //clk 1 X2 output
#define OutLO1  0 //clk 0 X1 output
unsigned long frequency;
unsigned long VFOfrequency; // Fréquence final d'affichage et base pour le VFO
long sweeprate=50; // Vitesse de défilement du pot d'accord
unsigned long bande = 7000000L; // valeur par défaut 
int bandes[12]={28000,28000,28000, 24890,21000,18068,14000,10100,7000,5315,3500,1800}; // limites inférieures des bandes
unsigned long IF = 9000000L;
unsigned long correctionfactor =0; //différence constatée entre affichage de la fréquence et la mesure.
// *************  SI5315 routines - tks Jerry Gaffke, KE7ER   ***********************

// An minimalist standalone set of Si5351 routines.
// VCOA is fixed at 875mhz, VCOB not used.
// The output msynth dividers are used to generate 3 independent clocks
// with 1hz resolution to any frequency between 4khz and 109mhz.

// Usage:
// Call si5351bx_init() once at startup with no args;
// Call si5351bx_setfreq(clknum, freq) each time one of the
// three output CLK pins is to be updated to a new frequency.
// A freq of 0 serves to shut down that output clock.

// The global variable si5351bx_vcoa starts out equal to the nominal VCOA
// frequency of 25mhz*35 = 875000000 Hz.  To correct for 25mhz crystal error,
// the user can adjust this value.  The vco frequency will not change but
// the number used for the (a+b/c) output msynth calculations is affected.
// Example:  We call for a 5mhz signal, but it measures to be 5.001mhz.
// So the actual vcoa frequency is 875mhz*5.001/5.000 = 875175000 Hz,
// To correct for this error:     si5351bx_vcoa=875175000;

// Most users will never need to generate clocks below 500khz.
// But it is possible to do so by loading a value between 0 and 7 into
// the global variable si5351bx_rdiv, be sure to return it to a value of 0
// before setting some other CLK output pin.  The affected clock will be
// divided down by a power of two defined by  2**si5351_rdiv
// A value of zero gives a divide factor of 1, a value of 7 divides by 128.
// This lightweight method is a reasonable compromise for a seldom used feature.

void si5351bx_init() {                  // Call once at power-up, start PLLA
  uint8_t reg;  uint32_t msxp1;
  Wire.begin();
  i2cWrite(149, 0);                     // SpreadSpectrum off
  i2cWrite(3, si5351bx_clken);          // Disable all CLK output drivers
  i2cWrite(183, SI5351BX_XTALPF << 6);  // Set 25mhz crystal load capacitance
  msxp1 = 128 * SI5351BX_MSA - 512;     // and msxp2=0, msxp3=1, not fractional
  uint8_t  vals[8] = {0, 1, BB2(msxp1), BB1(msxp1), BB0(msxp1), 0, 0, 0};
  i2cWriten(26, vals, 8);               // Write to 8 PLLA msynth regs
  i2cWrite(177, 0x20);                  // Reset PLLA  (0x80 resets PLLB)
  // for (reg=16; reg<=23; reg++) i2cWrite(reg, 0x80);    // Powerdown CLK's
  // i2cWrite(187, 0);                  // No fannout of clkin, xtal, ms0, ms4
}

void si5351bx_setfreq(uint8_t clknum, uint32_t fout) {  // Set a CLK to fout Hz
  uint32_t  msa, msb, msc, msxp1, msxp2, msxp3p2top;
  if ((fout < 500000) || (fout > 109000000)) // If clock freq out of range
    si5351bx_clken |= 1 << clknum;      //  shut down the clock
  else {
    msa = si5351bx_vcoa / fout;     // Integer part of vco/fout
    msb = si5351bx_vcoa % fout;     // Fractional part of vco/fout
    msc = fout;             // Divide by 2 till fits in reg
    while (msc & 0xfff00000) {
      msb = msb >> 1;
      msc = msc >> 1;
    }
    msxp1 = (128 * msa + 128 * msb / msc - 512) | (((uint32_t)si5351bx_rdiv) << 20);
    msxp2 = 128 * msb - 128 * msb / msc * msc; // msxp3 == msc;
    msxp3p2top = (((msc & 0x0F0000) << 4) | msxp2);     // 2 top nibbles
    uint8_t vals[8] = { BB1(msc), BB0(msc), BB2(msxp1), BB1(msxp1),
                        BB0(msxp1), BB2(msxp3p2top), BB1(msxp2), BB0(msxp2)
                      };
    i2cWriten(42 + (clknum * 8), vals, 8); // Write to 8 msynth regs
    i2cWrite(16 + clknum, 0x0C | si5351bx_drive[clknum]); // use local msynth
    si5351bx_clken &= ~(1 << clknum);   // Clear bit to enable clock
  }
  i2cWrite(3, si5351bx_clken);        // Enable/disable clock
}

void i2cWrite(uint8_t reg, uint8_t val) {   // write reg via i2c
  Wire.beginTransmission(SI5351BX_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void i2cWriten(uint8_t reg, uint8_t *vals, uint8_t vcnt) {  // write array
  Wire.beginTransmission(SI5351BX_ADDR);
  Wire.write(reg);
  while (vcnt--) Wire.write(*vals++);
  Wire.endTransmission();
}
// *********** End of Jerry's si5315bx routines *********************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateDisplay() // Routine de raz du display
{
    myOLED.clrScr();
  myOLED.print(VERSION, LEFT, 0);
 myOLED.printNumF(VFOfrequency/1000,2,CENTER, 40); //Impression du nombre flottant VFOfrequency. Mais pour l'instant c'est un entier...
  myOLED.update();
    //lcd.setCursor(4,1);
    //lcd.print(VFOfrequency+correctionfactor);//fréquence corrigée
}
void doTuning() // Routine d'accord et de réglage de la bande
{ 
 byte i;
    //int t;    
    // int s;
     //int NumBande;
     int LimInfBande; 
  for (i = 0; i < nombreech; i++){
    s = analogRead(ANALOG_TUNING); // s valeur brute à lisser lue sur le potentiomètre
    ech[i]=s; //analogRead(ANALOG_TUNING); Putaing ! Le tableau commence par i(0)!!!
    //total = (total+ech[i]);
    //total =total/12;//-ech[i-1];
    //Serial.print(" VFOfrequency ");
    //Serial.print(ech[i]);
    Serial.print("i");
    Serial.print(i);
    Serial.print("s ");
    Serial.print(s);
    Serial.print("total ");
    Serial.println(total);
    //    Serial.println(total);
    /*Serial.print(VFOfrequency);
    Serial.print(" s ");
    Serial.print(s);
  
    //Serial.print (ech[6]);
    for (i = 0; i < nombreech; i++){
    s=ech[1]+ech[2]+ech[3]+ech[4]+ech[5]+ech[6]+ech[7]+ech[8]+ech[9]+ech[10]+ech[11];//+ech[12];                                                 )/10;
    s=s/11; //}
    Serial.print (" somme ");
    Serial.println(s);
   */ delay(300);

   /*
   const int nombreech=5;
int ech[nombreech];
int indice=0;
int total=0;
int moyenne=0;
    */
    //delay(100);
    //t= analogRead(BAND); //sample tunning pot
    //NumBande = int (t/92);    
    //lcd.setCursor(1,0);//afac vérification de s
    //lcd.print(NumBande);//afac
    LimInfBande = 7000;
   //LimInfBande = bandes[NumBande];
    VFOfrequency = 1000L*LimInfBande+(s*sweeprate); // Calcul la fréquence, affichage OK
    }
    }

void setFrequency(unsigned long freqset)
{
si5351bx_setfreq(OutLO1, freqset);
}

void setup() {
  if(!myOLED.begin(SSD1306_128X64))
    while(1);   // In case the library failed to allocate enough RAM for the display buffer...
    myOLED.setFont(BigNumbers);
Serial.begin(9600);

  for (int i = 0; i < nombreech; i++) {
    ech[i] = 0;
}

si5351bx_init();
  //lcd.begin(16, 2);
  //lcd.print(F("Version 2023.51"));
  //lcd.setCursor(0, 1);
  //lcd.print(F("Rx F6GMQ"));
  delay(1000);
  //lcd.clear();
}

void loop() {
doTuning();
updateDisplay();
frequency = VFOfrequency;
setFrequency(frequency);
Serial.println(frequency);

}


