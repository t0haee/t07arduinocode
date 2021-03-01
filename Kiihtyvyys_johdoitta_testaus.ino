// Määritellään kytkentänavat kiihtyvyysanturille:
const int VccPin2 = A0;  // Käyttöjännite
const int xPin   = A1;   // x-kanavan mittaus
const int yPin   = A2;   // y-kanava
const int zPin   = A3;   // z-kanava
const int GNDPin2 = A4;  // laitteen maa-napa

// Muuttujamäärittelyt. Huomaa, että desimaalierotin on piste!
unsigned long aika = 0; // Aikaleima (ms), tyyppinä "pitkä, merkitön" muoto, koska INT-tyyppisenä numeroavaruus tulee n. puolessa minuutissa täyteen.
unsigned long mennyt_aika = 0;

int x = 0; //x-kanavan sensoriarvo 0 - 1023
int y = 0;
int z = 0;

int SisaanTunniste = 0;
int indeksi = 0;

int toisto = 0;
boolean isUp = false;

float ax = 0.0;  // x-kanavan kiihtyvyysarvo SI-muodossa (m/s^2)
float ay = 0.0;
float az = 0.0;
float axyz = 0.0;

float ax_out = 0.0;
float ay_out = 0.0;
float az_out = 0.0;
float axyz_out = 0.0;

float ax2 = 0.0;
float ay2 = 0.0;
float az2 = 0.0;
float axyz2 = 0.0;

float vx = 0.0;
float vy = 0.0;
float vz = 0.0;
float vxyz = 0.0;

float vx1 = 0.0;
float vy1 = 0.0;
float vz1 = 0.0;
float vxyz1 = 0.0;

float vx2 = 0.0;
float vy2 = 0.0;
float vz2 = 0.0;
float vxyz2 = 0.0;

void setup() {
  Serial.begin (19200); // Tämä täytyy valita myös Serial Monitorista samaksi

  // Kiihtvyys-anturin napojen määrittely:
  pinMode(VccPin2, OUTPUT);     // Kiihtyvyysanturin käyttöjännite Vcc
  pinMode(GNDPin2, OUTPUT);     // Kiihtyvyysanturin GND

  // Asetetaan syöttöjännite (5V UNO-BOARDILLA, 3.3V Genuino 101:llä) ja maa-arvot (0V):
  digitalWrite(VccPin2, HIGH);
  delayMicroseconds(2);
  digitalWrite(GNDPin2, LOW);
  delayMicroseconds(2);

  while (Serial.available() != 0)
  {
    // Odotellaan että yhteys käynnistyy jos tässä sattuu olemaan viivettä. 0 tarkoittaa että yhteys on.
  }
}

void loop() {

  // eka sisäänmenolla annetaan 1ms aikaa käynnistyä. Muuten 1. arvo on pelkkää häiriötä.
  if (SisaanTunniste == 0)
  {
    delay(1); // 1ms viive käynnistymiselle
    SisaanTunniste = 1; // muutetaan testattava muuttuja jotta tänne ei enää tulla
  }

  // Aikaleima (ms)
  aika = millis(); // Aikaleima luetaan ennen laskentaa, tosi SERIAL PLOTTER EI KÄYTÄ TÄTÄ!

  x = analogRead(xPin);
  y = analogRead(yPin);
  z = analogRead(zPin);

  ax = 0.1422 * x - 48.857; //Kalibrointiyhtälö x-akselin sensoriarvosta x-suunnan kiihtyvyydeksi.
  ax_out = 0.05 * ax + 0.95 * ax_out;
  ax2 = ax - ax_out; //vähennetään keskiarvo pois

  ay = 0.1382 * y - 47.116;
  ay_out = 0.05 * ay + 0.95 * ay_out;
  ay2 = ay - ay_out; //vähennetään keskiarvo pois

  az = 0.1443 * z - 50.718;
  az_out = 0.05 * az + 0.95 * az_out;
  az2 = az - az_out;  //vähennetään keskiarvo pois

  axyz = sqrt(ax * ax + ay * ay + az * az);
  axyz_out = 0.05 * axyz + 0.95 * axyz_out;
  axyz2 = axyz - axyz_out; //vähennetään keskiarvo pois

  ax = ax - ax_out;
  ay = ay - ay_out;
  az = az - az_out;
  axyz = axyz - axyz_out;

  vx = vx + ax2 * (aika - mennyt_aika); //nopeusarvon seuranta
  vx1 = 0.95 * vx1 + 0.05 * vx; //vähennetään keskiarvo pois
  vx2 = vx - vx1; //poistetaan nopeudesta keskiarvo

  vy = vy + ay * (aika - mennyt_aika); //nopeusarvon seuranta
  vy1 = 0.95 * vy1 + 0.05 * vy; //vähennetään keskiarvo pois
  vy2 = vy - vy1; //poistetaan nopeudesta keskiarvo

  vz = vz + az * (aika - mennyt_aika); //nopeusarvon seuranta
  vz1 = 0.95 * vz1 + 0.05 * vz; //vähennetään keskiarvo pois
  vz2 = vz - vz1; //poistetaan nopeudesta keskiarvo

  vxyz = vxyz + axyz2 * (aika - mennyt_aika); //nopeusarvon seuranta
  vxyz1 = 0.95 * vxyz1 + 0.05 * vxyz; //vähennetään keskiarvo pois
  vxyz2 = vxyz - vxyz1; //poistetaan nopeudesta keskiarvo

  if ( vxyz > 4200 && isUp ) {
    isUp = false;
  }
  if ( vxyz < 3000 && vxyz != 0 && !isUp) {
    isUp = true;
    toisto ++;
  }

  Serial.print(aika); // poistetaan aikaleima jotta plotteri ei sekoa
  Serial.print(" "); // mutta jos aiot tehdä fysiikan laskentaa, niin aikaleima tarvitaan mukaan.
  Serial.print(vxyz);
  Serial.print(" ");
  Serial.print(toisto);
  Serial.println(" ");

  float(aika - mennyt_aika) / 1000;
  mennyt_aika = aika;

  //Viive ennen seuraavaa kierrosta niin dataa tulee kohtuullisella tahdilla
  // Jos mittaa hirmuisen nopeasti niin numerinen derivaatta näyttää kohinaisemmalta
  // vaikka siinä toki olisi tietoa enemmän!
  delay(2);
}
