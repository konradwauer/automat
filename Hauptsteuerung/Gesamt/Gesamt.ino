#include <Wire.h>

/*-------PIN-Definitionen-----------------------------------------------------------------*/

// Step-Pin (mit allen Motortreibern verbunden) mit PWM Signal belegt
const int pinSchritt = 6;
// Enable-Pins (jeweils mit Motortreibern verbunden)
const int pinsEnable[] = {46,45,44};

// Sensorpins: 
// Tisch 0 -> Port A: 22-27, 
// Tisch 1 -> Port F:A0-A5
// Tisch 2 -> Port K: A8-A13

// Periodendauer der Schrittmotoren bei Vorwaertsgang in ms
const int schrittPeriode = 2;
// Periodendauer der Sensorabfrageperiode
const int abtastPeriode = 4;
const int pinRichtung = 11;
const int pinEndlage[] = {5,10,2};
volatile uint16_t *OCR5[] = {&OCR5A,&OCR5B,&OCR5C};


const int entprellZyklen = 200;
int entprellZaehler[3][6] = {0};
uint8_t Sensorregister[] = {0,0,0};
const int schrittMaximum = 5000; // Maximale Schrittanzahl bis Tischende
int schrittKonto[3] = {schrittMaximum,schrittMaximum,schrittMaximum};
const int lochBonus[] = {300,300,300,600,600,1000};
const int lochReihenfolge [3][6] ={{2,5,1,3,0,4},{0,1,2,3,4,5},{0,1,2,3,4,5}};
boolean spielBeendet = true;
int siegerTisch = 3;


int startePWM_Pin6(int periodendauer_ms)
{
  TCCR4A = (1<<COM4A1); // Pin 6 verbinden, non-inverting
  TCCR4A |= (1<<WGM41); // Fast PWM Teil 1 (WGM4 = 14, letzte beiden Bits in TCCR4A)
  TCCR4B = (1<<WGM42) | (1<<WGM43); // Fast PWM Teil 2 (erste beiden Bits)
  TCCR4B |= (0b101<<CS40); //Prescaler 1024
  int prescaler = 1024;
  ICR4 = periodendauer_ms*((F_CPU)/prescaler)/1000; // Input Capture Register
  OCR4A = ICR4/2; // Output Compare Register
  return 1;
}
/*
Startet Timer 5 als Counter des PWM Signals an Pin 6. Dafür wird der Timer als externe Clocksource genutzt
Pin 6 muss dafür mit Pin 47 verbunden sein
*/
int startePWMcounter(void)
{
  TCCR5A = (1<<COM5A1) | (1<<COM5B1) | (1<<COM5C1); // Pins 46, 45 und 44 als Sleep-Pins verbinden
  TCCR5A &= ~ ((1<<WGM51) | (1<<WGM50)); // Normal alle WGM5=0
  TCCR5B = (0b110<<CS50); // External clock source (falling edge)
  ICR5 = 0xFFFF; // Input Capture Register maximal
  OCR5A = 0x0001; // Output Compare Register
  OCR5B = 0x0001;
  OCR5C = 0x0001;
  TCNT5 = 0x0000; // Timer Counter Register: Zähler auf Null setzen
}

/*
Startet Timer 3. Dieser dient dem Einstellen einer konstanten Abtastrate.
*/
void starteAbtastTimer(int abtastPeriode_ms)
{
  TCCR3A = 0x0000; // Keine Pins verbunden
  TCCR3B = (1 << WGM33) | (1 << WGM32); // CTC Mode mit ICR3 als Top
  TCCR3B |= (1 << CS30); // Prescaler 1 -> ohne Prescaler
  int prescaler = 1;
  ICR3 = abtastPeriode_ms*((F_CPU)/prescaler)/1000; // Input Capture Register
}

void initialisiereSensorpins(void)
{
  DDRA &= 0xC0; // Pins als Input
  DDRF &= 0xC0;
  DDRK &= 0xC0;
  PORTA &= 0xC0;
  PORTF &= 0xC0;
  PORTK &= 0xC0;

  PORTA = 0xFF; //TODO !!!!!!!!!!!!! NUR ZUM TESTEN: PULLUP AKTIVIERT !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  PORTF = 0xFF;
  PORTK = 0xFF;

}

void pruefeLoecher(void)
{
  Sensorregister[0] = (~PINA) & 0x3F; // eventuell an Sensorlogik anpassen
  Sensorregister[1] = (~PINF) & 0x3F;
  Sensorregister[2] = (~PINK) & 0x3F;
  
  for(int tisch = 0; tisch <= 2; tisch++)
  {
    for(int loch = 0; loch <=5; loch++)
    {
      if(Sensorregister[tisch] & (1 << loch))
      {
        if(entprellZaehler[tisch][loch] <= 0)
        {
          // Loch erstmalig detektiert
          entprellZaehler[tisch][loch] = entprellZyklen;
          Serial.print(loch);Serial.print(" "); 
          Serial.println(tisch);
          setzeSchritte(tisch, lochReihenfolge[tisch][loch]);
          sendeSpielstand(tisch,lochReihenfolge[tisch][loch]);
        }
      }
      else if(entprellZaehler[tisch][loch] > 0)
      {
        // Loch wurde in den letzten Entprellzyklen getroffen
        entprellZaehler[tisch][loch] -=1;
      }      
    }
  }
}

void setzeSchritte(int tisch, int loch)
{
  int schrittDifferenz = schrittMaximum - schrittKonto[tisch];
  
  if(schrittDifferenz > lochBonus[loch])
  {
    // Normalfall
    addiereSchritte(lochBonus[loch], tisch);
    schrittKonto[tisch] += lochBonus[loch];
  }
  else
  {
    // Dieser Tisch ist Sieger
    addiereSchritte(schrittDifferenz, tisch);
    siegerTisch = tisch;
    spielBeendet = true;
    schrittKonto[tisch] += schrittDifferenz;
    sendeSpielstand(tisch, 6);
  }
  
  Serial.println(schrittKonto[tisch]);
}
    

void addiereSchritte(int anzahlSchritte, int tisch)
{
  Serial.println("addiere Schritte");Serial.println(anzahlSchritte);Serial.println(tisch);
  // TODO: tisch beachten!!!
  // Beachten Tischreihenfolge "umgekehrt": Tisch 0 an Channel C
  int aktuellerCounterWert; 
  if(TIFR5 & (0x02<<tisch))
  {
    // Compare Match Flag ist high -> ~Enable High -> Motor steht
    
    aktuellerCounterWert = TCNT5;
    *OCR5[tisch] = anzahlSchritte + aktuellerCounterWert;
    // Motor aktivieren
    TCCR5A |=  (0xC0 >> (2*tisch)); // Set on compare match
    TCCR5C |= (0x80 >> tisch); // Force output compare
    TCCR5A &= ~ (0x40 >> (2*tisch)); // Clear on compare match
    TIFR5 = (0x02 << tisch); //Output Compare Match Flag
  }
  else
  {
    // Compare Match Flag ist low -> ~Enable Loq -> Motor läuft bereits
    *OCR5[tisch] += anzahlSchritte;
  }
}

void rueckfahrt()
{
  Serial.println("RWFahrt");
  
  digitalWrite(pinRichtung, HIGH);
  
  for(int tisch=0; tisch <= 2; tisch ++)
  {
    addiereSchritte(schrittKonto[tisch]*1.025, tisch);
  }
  while((TIFR5 & 0x0E) != 0x0E)
  {
    
    for(int tisch=0; tisch <= 2; tisch ++)
    {
      if(digitalRead(pinEndlage[tisch]) == 0)
      {
        //Serial.print("Force kompare irgendwas ganz langes, admit es auffällt");
        // Figur dieses Tisches ist an Endlage angekommen
        TCCR5C |= (0x80 >> tisch); // Force output compare
      }
    }
    //Serial.println();
  }
  Serial.println("RWFahrt beendet");
  digitalWrite(pinRichtung, LOW);
}

void sendeSpielstand(int tisch, int loch)
{
  //Kodierung: Loch6 --> Siegertisch steht fest
  //           Loch7 --> Siegertisch angekommen
  //           Loch6 --> Tisch5 Rückfahrt start
  //           Loch6 --> Tisch6 Rückfahrt beendet
  //           Loch7 --> Tisch5 Ampel rot
  //           Loch7 --> Tisch6 Ampel gelb
  //           Loch7 --> Tisch7 Ampel grün
  
  Wire.beginTransmission(1);  
  Wire.write(tisch);
  Wire.write(loch);
  Wire.write(schrittKonto[0]>>8);
  Wire.write(schrittKonto[0]);
  Wire.write(schrittKonto[1]>>8);
  Wire.write(schrittKonto[1]);
  Wire.write(schrittKonto[2]>>8);
  Wire.write(schrittKonto[2]);
  Wire.endTransmission();  
}

void setup()
{
  startePWM_Pin6(schrittPeriode);
  startePWMcounter();
  starteAbtastTimer(abtastPeriode);
  
  initialisiereSensorpins();
  pinMode(pinSchritt,OUTPUT);
  pinMode(pinRichtung,OUTPUT);
  for(int tisch = 0; tisch <= 2; tisch++)
  {
    pinMode(pinsEnable[tisch],OUTPUT);
    pinMode(pinEndlage[tisch], INPUT_PULLUP);
    //geändert07.04
    TCCR5A |=  (0xC0 >> (2*tisch)); // Set on compare match
    TCCR5C |= (0x80 >> tisch); // Force output compare
    TCCR5A &= ~ (0x40 >> (2*tisch)); // Clear on compare match
    TIFR5 = (0x02 << tisch); //Output Compare Match Flag    
  }
  
  Wire.begin();
  Serial.begin(9600);
}


void loop()
{
  //startePWMcounter();
  
  sendeSpielstand(5,6); //starte Rückfahrt
  rueckfahrt();
  sendeSpielstand(6,6); //Rückfahrt beendet
  
  spielBeendet = false;
  for(int tisch=0; tisch <= 2; tisch ++)
  {
    schrittKonto[tisch] = 0;
  }

  sendeSpielstand(5,7); //Ampel rot
  delay(3000);
  sendeSpielstand(6,7); //Ampel gelb
  delay(1000);
  sendeSpielstand(7,7); //Ampel grün && Spielstart
  
  while(! spielBeendet)
  {
    pruefeLoecher();
    while( ! (TIFR3 & (1<<ICF4)) );
    TIFR3 = 1<<ICF4;
  }
  Serial.print("Sieger: ");Serial.println(siegerTisch);
  while(! (TIFR5 & (0x2 << siegerTisch))){} // Warte solange bis Sieger angekommen ist
  Serial.println(TIFR5);
  TCCR5C |= 0xE0; // Motoren deaktivieren  
  sendeSpielstand(siegerTisch,7); //Schicke Sieger angekommen

  delay(2000);
}
