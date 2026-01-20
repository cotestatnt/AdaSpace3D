/** Project CPP includes. */
#include "src/TLx493D/TLx493D_inc.hpp"
#include <math.h>

#define I2C_SDA 14
#define I2C_SCL 15

using namespace ifx::tlx493d;

TLx493D_A1B6 dut(Wire1, TLx493D_IIC_ADDR_A0_e);


/** Definition of a counter variable. */
uint8_t count = 0;


// Offset di calibrazione iniziale (in LSB dei valori raw)
double offsetX = 0.0;
double offsetY = 0.0;
double offsetZ = 0.0;

// Filtri low-pass (media mobile esponenziale)
double filtX = 0.0;
double filtY = 0.0;
double filtZ = 0.0;
const double alpha = 0.05; // coefficiente di filtro (0<alpha<=1)

// Ultimi valori raw letti dal sensore
int16_t rawX = 0;
int16_t rawY = 0;
int16_t rawZ = 0;


void computeAnglesFromMagneticVector(double bx, double by, double bz,
                                     double* pitchDeg,
                                     double* rollDeg,
                                     double* yawDeg) {
    // Mantiene in memoria l'ultimo angolo valido per evitare salti
    static double lastPitchDeg = 0.0;
    static double lastRollDeg  = 0.0;
    static double lastYawDeg   = 0.0;

    const double eps  = 1e-9;  // per sicurezza numerica
    const double minB = 5.0;   // soglia minima sul modulo (in LSB filtrati)

    double B = sqrt(bx * bx + by * by + bz * bz);

    // Se il vettore è praticamente nullo, gli angoli sono indefiniti:
    // in questo caso manteniamo l'ultimo valore stabile.
    if (B < minB) {
        *pitchDeg = lastPitchDeg;
        *rollDeg  = lastRollDeg;
        *yawDeg   = lastYawDeg;
        return;
    }

    // Angoli "sferici" più stabili:
    //  - yaw   (azimuth) nel piano X-Y: atan2(Y, X)
    //  - pitch (elevazione): atan2(Z, sqrt(X^2 + Y^2))
    //  - roll  non osservabile da un singolo vettore: lo fissiamo a 0

    double horiz = sqrt(bx * bx + by * by);
    if (horiz < eps) {
        horiz = eps;
    }

    double yawRad   = atan2(by, bx);          // [-pi, pi]
    double pitchRad = atan2(bz, horiz);       // [-pi/2, pi/2]

    double yawDegLoc   = yawRad   * 180.0 / PI;
    double pitchDegLoc = pitchRad * 180.0 / PI;
    double rollDegLoc  = 0.0;                 // roll non definibile da solo con un magnetometro

    // Aggiorna gli angoli di uscita e memorizza come ultimi valori validi
    *yawDeg   = yawDegLoc;
    *pitchDeg = pitchDegLoc;
    *rollDeg  = rollDegLoc;

    lastYawDeg   = yawDegLoc;
    lastPitchDeg = pitchDegLoc;
    lastRollDeg  = rollDegLoc;
}


void calibrateSensorOffset() {
    const int samples = 100;
    double sumX = 0.0;
    double sumY = 0.0;
    double sumZ = 0.0;
    int16_t tmpX, tmpY, tmpZ;
    int validSamples = 0;

    Serial.println("Inizio calibrazione: tieni il campo a zero...");
    delay(1000);

    for (int i = 0; i < samples; ++i) {
        if (dut.getRawMagneticField(&tmpX, &tmpY, &tmpZ)) {
            sumX += static_cast<double>(tmpX);
            sumY += static_cast<double>(tmpY);
            sumZ += static_cast<double>(tmpZ);
            ++validSamples;
        }

        delay(10);
    }

    if (validSamples > 0) {
        offsetX = sumX / validSamples;
        offsetY = sumY / validSamples;
        offsetZ = sumZ / validSamples;
    }

    Serial.println("Calibrazione completata.");
    Serial.print("Offset X: "); Serial.println(offsetX);
    Serial.print("Offset Y: "); Serial.println(offsetY);
    Serial.print("Offset Z: "); Serial.println(offsetZ);
}


void setup() {
    Wire1.setSCL(I2C_SCL);
    Wire1.setSDA(I2C_SDA);
    Wire1.begin();

    Serial.begin(115200);
    while (!Serial) {
        delay(10); // wait for serial port to connect. Needed for native USB
    }
    Serial.print("TLx493D Test\n");
    
    dut.begin();
    dut.setSensitivity(TLx493D_FULL_RANGE_e);

    // Calibrazione iniziale per azzerare l'offset
    calibrateSensorOffset();

    Serial.print("Registers: ");
    dut.printRegisters();

    Serial.print("setup done.\n");
}


/** In the loop we continuously reading the temperature value as well as the
 *  magnetic values in X, Y, Z-direction of the sensor and printing them to
 *  the serial monitor
 */
void loop() {
    
    
    if (!dut.getRawMagneticField(&rawX, &rawY, &rawZ) ) {
        Serial.println("getRawMagneticField error\n");
        return;
    }

    // Converte in double e applica l'offset di calibrazione
    double x = static_cast<double>(rawX) - offsetX;
    double y = static_cast<double>(rawY) - offsetY;
    double z = static_cast<double>(rawZ) - offsetZ;

    // Inizializza il filtro al primo ciclo
    static bool filterInitialized = false;
    if (!filterInitialized) {
        filtX = x;
        filtY = y;
        filtZ = z;
        filterInitialized = true;
    }

    // Aggiorna il filtro low-pass (media mobile esponenziale)
    filtX = filtX + alpha * (x - filtX);
    filtY = filtY + alpha * (y - filtY);
    filtZ = filtZ + alpha * (z - filtZ);

    double pitch = 0.0;
    double roll  = 0.0;
    double yaw   = 0.0;

    // Calcola gli angoli a partire dal vettore magnetico filtrato
    computeAnglesFromMagneticVector(filtX, filtY, filtZ, &pitch, &roll, &yaw);

    // Converte i valori filtrati di nuovo in interi (LSB)
    int16_t outX = static_cast<int16_t>(round(filtX));
    int16_t outY = static_cast<int16_t>(round(filtY));
    int16_t outZ = static_cast<int16_t>(round(filtZ));

    // Stampa CSV: Xraw_corr, Yraw_corr, Zraw_corr, pitch, roll, yaw
    Serial.print(outX);
    Serial.print(", ");
    Serial.print(outY);
    Serial.print(", ");
    Serial.print(outZ);
    Serial.print(", ");
    Serial.print(pitch);
    Serial.print(", ");
    Serial.print(roll);
    Serial.print(", ");
    Serial.println(yaw);

    // Nessun reset periodico: evitiamo salti improvvisi nelle letture

    delay(40); // ~25 campioni/s: sufficiente e con buona stabilità
}
