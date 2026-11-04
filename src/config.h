/**CONFIGURACIÓN DE CONSTANTES
*/

// ==========================================
// Pines y direcciones 
// ==========================================

// Pines I2C estándar para la ESP32
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// Dirección I2C del módulo PCA9685 (por defecto: 0x40)
constexpr uint8_t DIRECCION_PCA9685 = 0x40;


// Asignación de canales del PCA9685 a cada articulación  
enum Articulacion {
    BASE = 0,     
    HOMBRO = 1,  
    CODO = 2,    
    PINZA = 3,   
    CANTIDAD_SERVOS = 4
};

// Pine del senror HC-SR04 y umbral
const int TRIG_PIN = 18;
const int ECHO_PIN = 33;
const float UMBRAL_CM = 8.5f;                                   // AJUSTAR: menor que esto = hay bloque (se calibra con mediciones reales)

// ==========================================
// Geometrìa **
// ==========================================

// longitud de eslabones en mm
float LONGITUD_ESLABON[3] = {84.005f, 70.3f, 82.36f};             // AJUSTAR: diferencia cn impresoin 3d ?

// altura del hombro en mm
const float ALTURA_HOMBRO = 40.0f;                               // AJUSTAR:

// tamaño cubo en mm
const int DIMENSION_CUBO = 50

// ==========================================
// Calibraciòn **
// ==========================================

// offset y signo de los servos                                                    // AJUSTAR:

// rango de movimiento de servos

// Configuración de pulsos PWM para servomotores 
constexpr int PULSO_MINIMO = 125;  // Equivalente aprox. a 0°
constexpr int PULSO_MAXIMO = 575;  // Equivalente aprox. a 180°
constexpr int FRECUENCIA_PWM = 50; // Frecuencia en Hz para servos
// àngulo de pinza abierta y cerrada(º)
const int PINZA_ABIERTA = 40                                                         // AJUSTAR: para 50mm
const int PINZA_CERRADA = 90

// ==========================================
// Puntos conocidos **
// ==========================================
//recogida                                                                       // AJUSTAR:
// deposito 
//home
//pose de escaneo 


// ==========================================
// Paràmetros de movimiento **
// ==========================================

// Velocidades 
constexpr int PAUSA_PASO_LIBRE_MS = 15;      // Movimiento sin carga
constexpr int PAUSA_PASO_TRANSPORTE_MS = 25; // Movimiento transportando el cubo
constexpr int PAUSA_AGARRE_MS = 400;         // Espera tras abrir/cerrar la pinza

// Altura de aproximaciòn                                                      // AJUSTAR:
