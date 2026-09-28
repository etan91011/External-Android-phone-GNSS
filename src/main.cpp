#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // GNSS
#include <MPU9250_WE.h>                           // IMU
#include <SdFat.h>                                // exFAT/FAT
#include <TimeLib.h>                              // time_t
#include <Timezone.h>                             // local TZ conversion

// -------------------- User config --------------------
static constexpr uint8_t GNSS_NAV_HZ = 10; // 10 Hz logging rate (GNSS-driven)

// SD chip select (change this if SD init fails!)
static constexpr uint8_t SD_CS_PIN = 26;

// SPI speed for SD (start conservative; raise later if you want)
static constexpr uint8_t SD_SPI_MHZ = 12;

// IMU I2C address is usually 0x68 for MPU9250
static constexpr uint8_t IMU_ADDR = 0x68;
static bool imuOk = false;
static bool magOk = false;

// -----------------------------------------------------

// Chicago / US Central time rules
TimeChangeRule usCDT = {"CDT", Second, Sun, Mar, 2, -300}; // UTC-5
TimeChangeRule usCST = {"CST", First,  Sun, Nov, 2, -360}; // UTC-6
Timezone Chicago(usCDT, usCST);

SFE_UBLOX_GNSS gnss;
MPU9250_WE imu(&Wire, IMU_ADDR);

// SdFat "universal" filesystem: supports FAT + exFAT when SDFAT_FILE_TYPE=3
SdFs sd;
FsFile logFile;

static char logName[48] = {0};

static void blink(uint8_t times, uint16_t onMs = 80, uint16_t offMs = 120)
{
  for (uint8_t i = 0; i < times; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(onMs);
    digitalWrite(LED_BUILTIN, LOW);
    delay(offMs);
  }
}

// Option B: Print uint64_t in decimal without relying on Print::print(uint64_t)
static void printU64(Print &p, uint64_t v)
{
  char buf[21]; // max 20 digits for uint64_t + null terminator
  char *end = &buf[20];
  *end = '\0';

  do
  {
    *--end = char('0' + (v % 10));
    v /= 10;
  } while (v);

  p.print(end);
}

static bool openUniqueLogFile(const char *basePrefix)
{
  // Try log_0000.csv ... log_9999.csv (exFAT supports long names fine)
  for (uint16_t i = 0; i < 10000; i++)
  {
    snprintf(logName, sizeof(logName), "%s_%04u.csv", basePrefix, i);
    if (!sd.exists(logName))
    {
      logFile = sd.open(logName, O_WRONLY | O_CREAT);
      return (bool)logFile;
    }
  }
  return false;
}

static void writeHeader()
{
  // Keep it explicit: units included in names
  logFile.println(
    F("unix_ms,"
      "local_iso,"
      "tz,"
      "uptime_ms,"
      "fixType,"
      "gnssFixOk,"
      "diffSoln,"
      "rtkCarrierSoln,"
      "siv,"
      "lat_deg,"
      "lon_deg,"
      "alt_m_ellipsoid,"
      "alt_m_msl,"
      "velN_mps,"
      "velE_mps,"
      "velD_mps,"
      "groundSpeed_mps,"
      "heading_deg,"
      "pdop,"
      "hAcc_m,"
      "vAcc_m,"
      "speedAcc_mps,"
      "headingAcc_deg,"
      "headVeh_deg,"
      "magDec_deg,"
      "magAcc_deg,"
      "imu_ax_g,"
      "imu_ay_g,"
      "imu_az_g,"
      "imu_gx_dps,"
      "imu_gy_dps,"
      "imu_gz_dps,"
      "imu_mx_uT,"
      "imu_my_uT,"
      "imu_mz_uT,"
      "imu_temp_C"
    )
  );
  logFile.flush();
}

static void formatLocalTime(time_t utcSec, uint16_t ms, char *out, size_t outSz, const char **tzNameOut)
{
  TimeChangeRule *tcr = nullptr;
  time_t local = Chicago.toLocal(utcSec, &tcr);
  const char *tzName = (tcr != nullptr) ? tcr->abbrev : "UTC";
  if (tzNameOut) *tzNameOut = tzName;

  // yyyy-mm-dd hh:mm:ss.mmm
  snprintf(out, outSz, "%04d-%02d-%02d %02d:%02d:%02d.%03u",
           year(local), month(local), day(local),
           hour(local), minute(local), second(local),
           (unsigned)ms);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  delay(200);

  Wire.begin();
  Wire.setClock(400000); // faster I2C helps at higher GNSS rates

  Serial.println(F("\nWisBlock logger starting..."));
  blink(2);

  // ---- GNSS init (RAK12500) ----
  // SparkFun example uses begin() on default I2C address and then:
  // setI2COutput(COM_TYPE_UBX), setNavigationFrequency, setAutoPVT(true).
  if (!gnss.begin())
  {
    Serial.println(F("GNSS not detected on I2C. Check RAK12500 seating / power."));
    blink(10, 40, 60);
    while (1) delay(100);
  }

  gnss.setI2COutput(COM_TYPE_UBX);              // reduce I2C noise/bandwidth
  gnss.setNavigationFrequency(GNSS_NAV_HZ);     // request NAV rate (Hz)
  gnss.setAutoPVT(true);                        // auto NAV-PVT

  // ---- IMU init (MPU9250 / “RAK 9-axis IMU”) ----
  // MPU9250_WE is commonly used for WisBlock MPU9250 setups.
imuOk = imu.init();
if (!imuOk)
{
  Serial.println(F("IMU not detected at 0x68. Check module / slot / I2C."));
}
else
{
  // IMPORTANT for mag: do this once
  magOk = imu.initMagnetometer();
  if (!magOk)
  {
    Serial.println(F("Magnetometer not detected (expected AK8963 at 0x0C). Mag will log as blank."));
  }

  imu.autoOffsets();
  imu.setAccRange(MPU9250_ACC_RANGE_4G);
  imu.setGyrRange(MPU9250_GYRO_RANGE_250);

  if (magOk)
    imu.setMagOpMode(AK8963_CONT_MODE_100HZ);
}


  // ---- SD init (RAK15002) ----
  Serial.println(F("Initializing SD..."));
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  if (!sd.begin(SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(SD_SPI_MHZ))))
  {
    Serial.println(F("SD begin FAILED."));
    Serial.println(F("Most common causes: wrong CS pin, card not seated, or power issue."));
    blink(20, 30, 30);
    while (1) delay(100);
  }

  // Start with a boot log name; later we’ll still keep logging even before time fix
  if (!openUniqueLogFile("boot"))
  {
    Serial.println(F("Could not create log file."));
    blink(20, 30, 30);
    while (1) delay(100);
  }

  writeHeader();

  Serial.print(F("Logging to: "));
  Serial.println(logName);
  blink(3);
}

void loop()
{
  // Keep GNSS parsing moving
  gnss.checkUblox();

  // Log only when a fresh NAV-PVT solution arrives (GNSS-driven “10 Hz”)
  if (!gnss.getPVT())
    return;

  // ----- Time -----
  bool dateValid = gnss.getDateValid();
  bool timeValid = gnss.getTimeValid();
  uint16_t ms = gnss.getMillisecond();
  uint32_t unixSec = 0;

  char localIso[32] = "";
  const char *tzName = "UNK";

  if (dateValid && timeValid)
  {
    unixSec = gnss.getUnixEpoch(); // seconds since 1970-01-01
    formatLocalTime((time_t)unixSec, ms, localIso, sizeof(localIso), &tzName);
  }

  uint64_t unixMs = (uint64_t)unixSec * 1000ULL + (uint64_t)ms;
  uint32_t uptimeMs = millis();

  // ----- GNSS fields -----
  uint8_t fixType = gnss.getFixType();                 
  uint8_t gnssFixOk = gnss.getGnssFixOk();             
  uint8_t diffSoln = gnss.getDiffSoln();               
  uint8_t rtkSoln = gnss.getCarrierSolutionType();     
  uint8_t siv = gnss.getSIV();                         

  bool llhInvalid = gnss.getInvalidLlh();              

  // Position (scaled integers from library)
  int32_t lat_e7 = llhInvalid ? 0 : gnss.getLatitude();     
  int32_t lon_e7 = llhInvalid ? 0 : gnss.getLongitude();    
  int32_t alt_mm = llhInvalid ? 0 : gnss.getAltitude();     
  int32_t alt_msl_mm = llhInvalid ? 0 : gnss.getAltitudeMSL();

  // Velocities etc (mm/s, deg*1e-5)
  int32_t velN_mms = gnss.getNedNorthVel();            
  int32_t velE_mms = gnss.getNedEastVel();             
  int32_t velD_mms = gnss.getNedDownVel();             
  int32_t gSpeed_mms = gnss.getGroundSpeed();          
  int32_t head_e5 = gnss.getHeading();                 

  uint16_t pdop_centi = gnss.getPDOP();                
  int32_t hAcc_mm = gnss.getHorizontalAccEst();        
  int32_t vAcc_mm = gnss.getVerticalAccEst();          
  uint32_t sAcc_mms = gnss.getSpeedAccEst();           
  uint32_t headAcc_e5 = gnss.getHeadingAccEst();       

  // Optional vehicle heading + magnetic declination if valid
int32_t headVeh_e5 = 0;
if (gnss.getHeadVehValid())
  headVeh_e5 = gnss.getHeadVeh();

// Read mag declination/accuracy independently
int16_t  magDec_centi = gnss.getMagDec();
uint16_t magAcc_centi = gnss.getMagAcc();


  // Convert to floats for CSV readability
  double lat_deg = (double)lat_e7 * 1e-7;
  double lon_deg = (double)lon_e7 * 1e-7;
  double alt_m = (double)alt_mm / 1000.0;
  double alt_msl_m = (double)alt_msl_mm / 1000.0;

  double velN_mps = (double)velN_mms / 1000.0;
  double velE_mps = (double)velE_mms / 1000.0;
  double velD_mps = (double)velD_mms / 1000.0;
  double gSpeed_mps = (double)gSpeed_mms / 1000.0;

  double heading_deg = (double)head_e5 * 1e-5;
  double pdop = (double)pdop_centi * 1e-2;
  double hAcc_m = (double)hAcc_mm / 1000.0;
  double vAcc_m = (double)vAcc_mm / 1000.0;
  double sAcc_mps = (double)sAcc_mms / 1000.0;
  double headAcc_deg = (double)headAcc_e5 * 1e-5;

  double headVeh_deg = (double)headVeh_e5 * 1e-5;
  double magDec_deg = (double)magDec_centi * 1e-2;
  double magAcc_deg = (double)magAcc_centi * 1e-2;

// ----- IMU -----
float ax = NAN, ay = NAN, az = NAN;
float gx = NAN, gy = NAN, gz = NAN;
float mx = NAN, my = NAN, mz = NAN;
float tC = NAN;

if (imuOk)
{
  xyzFloat acc = imu.getGValues();
  xyzFloat gyr = imu.getGyrValues();
  tC = imu.getTemperature();

  ax = acc.x; ay = acc.y; az = acc.z;
  gx = gyr.x; gy = gyr.y; gz = gyr.z;

  if (magOk)
  {
    xyzFloat mag = imu.getMagValues();
    mx = mag.x; my = mag.y; mz = mag.z;
  }
}


  // ----- Write CSV line -----
  // If LLH invalid, still log time + IMU, but lat/lon/alt will be 0.0
  printU64(logFile, unixMs);
  logFile.print(',');

  logFile.print(localIso[0] ? localIso : "");
  logFile.print(',');

  logFile.print(localIso[0] ? tzName : "");
  logFile.print(',');

  logFile.print(uptimeMs);
  logFile.print(',');

  logFile.print(fixType);
  logFile.print(',');

  logFile.print(gnssFixOk);
  logFile.print(',');

  logFile.print(diffSoln);
  logFile.print(',');

  logFile.print(rtkSoln);
  logFile.print(',');

  logFile.print(siv);
  logFile.print(',');

  logFile.print(lat_deg, 7);
  logFile.print(',');

  logFile.print(lon_deg, 7);
  logFile.print(',');

  logFile.print(alt_m, 3);
  logFile.print(',');

  logFile.print(alt_msl_m, 3);
  logFile.print(',');

  logFile.print(velN_mps, 3);
  logFile.print(',');

  logFile.print(velE_mps, 3);
  logFile.print(',');

  logFile.print(velD_mps, 3);
  logFile.print(',');

  logFile.print(gSpeed_mps, 3);
  logFile.print(',');

  logFile.print(heading_deg, 5);
  logFile.print(',');

  logFile.print(pdop, 2);
  logFile.print(',');

  logFile.print(hAcc_m, 3);
  logFile.print(',');

  logFile.print(vAcc_m, 3);
  logFile.print(',');

  logFile.print(sAcc_mps, 3);
  logFile.print(',');

  logFile.print(headAcc_deg, 5);
  logFile.print(',');

  logFile.print(headVeh_deg, 5);
  logFile.print(',');

  logFile.print(magDec_deg, 2);
  logFile.print(',');

  logFile.print(magAcc_deg, 2);
  logFile.print(',');

  logFile.print(ax, 6);
  logFile.print(',');

  logFile.print(ay, 6);
  logFile.print(',');

  logFile.print(az, 6);
  logFile.print(',');

  logFile.print(gx, 3);
  logFile.print(',');

  logFile.print(gy, 3);
  logFile.print(',');

  logFile.print(gz, 3);
  logFile.print(',');

  logFile.print(mx, 2);
  logFile.print(',');

  logFile.print(my, 2);
  logFile.print(',');

  logFile.print(mz, 2);
  logFile.print(',');

  logFile.println(tC, 2);

  // Flush periodically so you don’t lose a ton of data if power is cut
  // (10 Hz => 10 lines/sec; flushing every 1 sec is a decent compromise)
  static uint32_t lastFlush = 0;
  if (millis() - lastFlush >= 1000)
  {
    logFile.flush();
    lastFlush = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // heartbeat
  }
}
