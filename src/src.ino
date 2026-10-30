/**
 * @file Rutina_PickAndPlace_ESP32.ino
 * @brief Rutina unica de recoger un cubo (PLA 4cm, 15% relleno) en una posicion
 *        definida y depositarlo en otra, con 4 servomotores via PCA9685 + ESP32.
 * @author Emilio Rojas
 *
 * IMPORTANTE: todos los angulos marcados con [CALIBRAR] son valores de ejemplo.
 * Deben ajustarse al brazo real antes de la primera prueba con carga.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==========================================
// CONFIGURACIÓN DE HARDWARE Y CONSTANTES
// ==========================================

// Pines I2C estándar para la ESP32
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// Dirección I2C del módulo PCA9685 (por defecto: 0x40)
constexpr uint8_t DIRECCION_PCA9685 = 0x40;

// Configuración de pulsos PWM para servomotores (Frecuencia: 50Hz)
constexpr int PULSO_MINIMO = 125;  // Equivalente aprox. a 0°
constexpr int PULSO_MAXIMO = 575;  // Equivalente aprox. a 180°
constexpr int FRECUENCIA_PWM = 50; // Frecuencia en Hz para servos

// Asignación de canales del PCA9685 a cada articulación  [CONFIRMAR]
enum Articulacion {
    BASE = 0,     // Giro horizontal
    HOMBRO = 1,   // Elevación del brazo
    CODO = 2,     // Extensión del antebrazo
    PINZA = 3,    // Apertura / cierre de la pinza
    CANTIDAD_SERVOS = 4
};

// Velocidades (pausa entre pasos de 1°; mayor valor = más lento y suave)
constexpr int PAUSA_PASO_LIBRE_MS = 15;      // Movimiento sin carga
constexpr int PAUSA_PASO_TRANSPORTE_MS = 25; // Movimiento transportando el cubo
constexpr int PAUSA_AGARRE_MS = 400;         // Espera tras abrir/cerrar la pinza

// ==========================================
// POSES DE LA RUTINA  [CALIBRAR]
// ==========================================

// Ángulos de la pinza. El cubo mide 40 mm: la apertura debe superar ese ancho.
constexpr int PINZA_ABIERTA = 120;
constexpr int PINZA_CERRADA_CUBO = 70; // Cierre que sujeta el cubo sin forzar el servo

// Una pose = un ángulo por articulación
struct Pose {
    int angulos[CANTIDAD_SERVOS];
};

//                                      BASE HOMBRO CODO PINZA
constexpr Pose POSE_INICIO            = {{ 90,  90,   90, PINZA_ABIERTA }};
constexpr Pose POSE_RECOGER_ARRIBA    = {{ 45,  70,  100, PINZA_ABIERTA }};
constexpr Pose POSE_RECOGER           = {{ 45,  50,  120, PINZA_ABIERTA }};
constexpr Pose POSE_DEPOSITAR_ARRIBA  = {{ 135, 70,  100, PINZA_CERRADA_CUBO }};
constexpr Pose POSE_DEPOSITAR         = {{ 135, 50,  120, PINZA_CERRADA_CUBO }};

// ==========================================
// OBJETOS Y VARIABLES GLOBALES
// ==========================================

// Instancia del controlador PCA9685
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(DIRECCION_PCA9685);

// Estado actual del sistema (ángulo individual de cada servo)
Pose poseActual = POSE_INICIO;

// ==========================================
// DECLARACIÓN DE FUNCIONES (PROTOTIPOS)
// ==========================================
void inicializarSistema();
void procesarPuertoSerie();
void ejecutarRutinaPickAndPlace();
void cerrarPinza();
void abrirPinza();
void moverAPose(const Pose& destino, int pausaPasoMs);
void posicionarServo(int canal, int angulo);
int convertirAnguloAPwm(int angulo);

// ==========================================
// SETUP Y BUCLE PRINCIPAL
// ==========================================

void setup() {
    inicializarSistema();
}

void loop() {
    procesarPuertoSerie();
}

// ==========================================
// MÓDULOS DE INICIALIZACIÓN
// ==========================================

/**
 * @brief Inicializa los buses de comunicación (I2C y Serial) y lleva el brazo a la pose inicial.
 */
void inicializarSistema() {
    Serial.begin(115200);
    while (!Serial); // Espera opcional a que el monitor serie esté listo

    Wire.begin(PIN_SDA, PIN_SCL);

    pca9685.begin();
    pca9685.setPWMFreq(FRECUENCIA_PWM);

    // Posición inicial de seguridad: se escribe directo (el brazo parte de posición desconocida)
    for (int canal = 0; canal < CANTIDAD_SERVOS; canal++) {
        posicionarServo(canal, poseActual.angulos[canal]);
    }

    Serial.println("\n========================================");
    Serial.println("  BRAZO ROBOTICO: RUTINA PICK AND PLACE ");
    Serial.println("========================================");
    Serial.println("Comandos:  i = ejecutar rutina | h = volver a inicio");
}

// ==========================================
// MÓDULOS DE COMUNICACIÓN SERIE
// ==========================================

/**
 * @brief Lee los comandos ingresados a través del Monitor Serie.
 */
void procesarPuertoSerie() {
    if (Serial.available() > 0) {
        String entrada = Serial.readStringUntil('\n');
        entrada.trim();
        entrada.toLowerCase();

        if (entrada == "i") {
            ejecutarRutinaPickAndPlace();
        } else if (entrada == "h") {
            Serial.println("[INFO] Volviendo a posicion de inicio...");
            moverAPose(POSE_INICIO, PAUSA_PASO_LIBRE_MS);
        } else {
            Serial.println("[AVISO] Comando invalido. Usa 'i' (iniciar) o 'h' (inicio).");
        }
    }
}

// ==========================================
// MÓDULOS DE RUTINA
// ==========================================

/**
 * @brief Recoge el cubo en POSE_RECOGER y lo deposita en POSE_DEPOSITAR.
 */
void ejecutarRutinaPickAndPlace() {
    Serial.println("[INFO] Iniciando rutina pick and place...");

    // 1. Aproximación al cubo (sin carga)
    moverAPose(POSE_INICIO, PAUSA_PASO_LIBRE_MS);
    moverAPose(POSE_RECOGER_ARRIBA, PAUSA_PASO_LIBRE_MS);
    moverAPose(POSE_RECOGER, PAUSA_PASO_LIBRE_MS);

    // 2. Agarre
    cerrarPinza();

    // 3. Transporte (más lento por la carga)
    moverAPose(POSE_RECOGER_ARRIBA, PAUSA_PASO_TRANSPORTE_MS);
    moverAPose(POSE_DEPOSITAR_ARRIBA, PAUSA_PASO_TRANSPORTE_MS);
    moverAPose(POSE_DEPOSITAR, PAUSA_PASO_TRANSPORTE_MS);

    // 4. Liberación
    abrirPinza();

    // 5. Retirada y regreso a inicio (sin carga)
    moverAPose(POSE_DEPOSITAR_ARRIBA, PAUSA_PASO_LIBRE_MS);
    moverAPose(POSE_INICIO, PAUSA_PASO_LIBRE_MS);

    Serial.println("[INFO] Rutina finalizada.");
}

/**
 * @brief Cierra la pinza sobre el cubo manteniendo el resto de articulaciones fijas.
 */
void cerrarPinza() {
    Pose destino = poseActual;
    destino.angulos[PINZA] = PINZA_CERRADA_CUBO;
    moverAPose(destino, PAUSA_PASO_TRANSPORTE_MS);
    delay(PAUSA_AGARRE_MS);
}

/**
 * @brief Abre la pinza para soltar el cubo manteniendo el resto de articulaciones fijas.
 */
void abrirPinza() {
    Pose destino = poseActual;
    destino.angulos[PINZA] = PINZA_ABIERTA;
    moverAPose(destino, PAUSA_PASO_TRANSPORTE_MS);
    delay(PAUSA_AGARRE_MS);
}

// ==========================================
// MÓDULOS DE CINEMÁTICA Y CONTROL DE MOVIMIENTO
// ==========================================

/**
 * @brief Mueve los 4 servos de forma gradual y sincronizada hasta la pose destino.
 *        Todos los servos parten y llegan al mismo tiempo, cada uno a su propia velocidad.
 * @param destino Pose final deseada.
 * @param pausaPasoMs Pausa entre pasos de 1° de la articulación que más recorre.
 */
void moverAPose(const Pose& destino, int pausaPasoMs) {
    Pose inicio = poseActual;

    // El servo con mayor recorrido define la cantidad de pasos
    int pasosTotales = 0;
    for (int canal = 0; canal < CANTIDAD_SERVOS; canal++) {
        int recorrido = abs(destino.angulos[canal] - inicio.angulos[canal]);
        if (recorrido > pasosTotales) pasosTotales = recorrido;
    }

    for (int paso = 1; paso <= pasosTotales; paso++) {
        for (int canal = 0; canal < CANTIDAD_SERVOS; canal++) {
            int delta = destino.angulos[canal] - inicio.angulos[canal];
            int angulo = inicio.angulos[canal] + (delta * paso) / pasosTotales;
            posicionarServo(canal, angulo);
            poseActual.angulos[canal] = angulo;
        }
        delay(pausaPasoMs);
    }
}

/**
 * @brief Envía la señal PWM correspondiente al ángulo para un canal.
 * @param canal Canal del PCA9685 (0 a 3).
 * @param angulo Ángulo objetivo en grados.
 */
void posicionarServo(int canal, int angulo) {
    pca9685.setPWM(canal, 0, convertirAnguloAPwm(angulo));
}

/**
 * @brief Traduce grados angulares (0-180) a pulsos de ciclo de trabajo del PCA9685.
 * @param angulo Ángulo en grados.
 * @return int Valor de pulso PWM equivalente.
 */
int convertirAnguloAPwm(int angulo) {
    angulo = constrain(angulo, 0, 180);
    return map(angulo, 0, 180, PULSO_MINIMO, PULSO_MAXIMO);
}
