/**
 * @file Rutina_PickAndPlace_Segura_ESP32.ino
 * @brief Rutina Pick and Place usando el motor de movimiento individual funcional.
 * @author Emilio Rojas
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==========================================
// CONFIGURACIÓN DE HARDWARE Y CONSTANTES
// ==========================================

constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;
constexpr uint8_t DIRECCION_PCA9685 = 0x40;

constexpr int PULSO_MINIMO = 125;  
constexpr int PULSO_MAXIMO = 575;  
constexpr int FRECUENCIA_PWM = 50; 

constexpr int CANTIDAD_SERVOS = 4;
constexpr int PAUSA_PASO_MS = 25;   
constexpr int ANGULO_INICIAL = 80;

// Definición de canales de las articulaciones
enum Articulacion {
    BASE = 0,     
    HOMBRO = 1,   
    CODO = 2,     
    PINZA = 3     
};

// Ángulos útiles para la pinza
constexpr int PINZA_ABIERTA = 60;
constexpr int PINZA_CERRADA = 80;

// ==========================================
// OBJETOS Y VARIABLES GLOBALES
// ==========================================

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(DIRECCION_PCA9685);
int anguloActual[CANTIDAD_SERVOS];

// ==========================================
// DECLARACIÓN DE FUNCIONES (PROTOTIPOS)
// ==========================================
void inicializarSistema();
void posicionarServo(int canal, int angulo);
void procesarPuertoSerie();
void moverServoSuavemente(int canal, int destino);
void ejecutarRutinaPickAndPlace();
void volverAInicio();
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

void inicializarSistema() {
    Serial.begin(115200);
    
    unsigned long tiempoInicio = millis();
    while (!Serial && (millis() - tiempoInicio < 2000));

    Wire.begin(PIN_SDA, PIN_SCL);

    pca9685.begin();
    pca9685.setPWMFreq(FRECUENCIA_PWM);

    // Inicializar y posicionar servos en ángulo base de seguridad
    for (int canal = 0; canal < CANTIDAD_SERVOS; canal++) {
        anguloActual[canal] = ANGULO_INICIAL;
        posicionarServo(canal, ANGULO_INICIAL);
        delay(100); // Pausa escalonada de arranque
    }

    Serial.println("\n========================================");
    Serial.println("  RUTINA PICK AND PLACE (MODO SEGURO)   ");
    Serial.println("========================================");
    Serial.println("Comandos:");
    Serial.println("  'i' -> Ejecutar rutina completa");
    Serial.println("  'h' -> Volver a posicion inicial");
    Serial.println("  O usa el formato manual: <servo 0-3> <angulo 0-180>");
}

// ==========================================
// MÓDULOS DE COMUNICACIÓN SERIE
// ==========================================

void procesarPuertoSerie() {
    if (Serial.available() > 0) {
        String entrada = Serial.readStringUntil('\n');
        entrada.trim();
        entrada.toLowerCase();

        // 1. Comprobar comandos de rutina
        if (entrada == "i") {
            ejecutarRutinaPickAndPlace();
            return;
        } 
        if (entrada == "h") {
            volverAInicio();
            return;
        }

        // 2. Comprobar control manual tradicional (<canal> <angulo>)
        int separador = entrada.indexOf(' ');
        if (separador <= 0 || !isDigit(entrada[0])) {
            Serial.println("[AVISO] Comando invalido. Usa 'i', 'h' o formato '<servo> <angulo>'");
            return;
        }

        String textoAngulo = entrada.substring(separador + 1);
        textoAngulo.trim();

        if (textoAngulo.length() == 0 || !isDigit(textoAngulo[0])) {
            Serial.println("[AVISO] Formato invalido.");
            return;
        }

        int canal = entrada.substring(0, separador).toInt();
        int nuevoAngulo = textoAngulo.toInt();

        if (canal < 0 || canal >= CANTIDAD_SERVOS) {
            Serial.println("[ERROR] Servo fuera de rango (0-3).");
            return;
        }

        if (nuevoAngulo >= 0 && nuevoAngulo <= 180) {
            Serial.print("[INFO] Moviendo Servo ");
            Serial.print(canal);
            Serial.print(" a ");
            Serial.println(nuevoAngulo);

            moverServoSuavemente(canal, nuevoAngulo);
        } else {
            Serial.println("[ERROR] Angulo fuera de rango (0-180).");
        }
    }
}

// ==========================================
// RUTINA DE PICK AND PLACE SECUENCIAL
// ==========================================

void ejecutarRutinaPickAndPlace() {
    Serial.println("\n[INFO] === INICIANDO RUTINA PICK AND PLACE ===");

    // 1. Abrir pinza y posicionarse sobre el objeto
    Serial.println("[Paso 1] Abriendo pinza y aproximando...");
    moverServoSuavemente(PINZA, PINZA_ABIERTA);
    moverServoSuavemente(BASE, 160);
    moverServoSuavemente(HOMBRO, 70);
    moverServoSuavemente(CODO, 70);

    // 2. Bajar a recoger el cubo
    Serial.println("[Paso 2] Bajando a por el cubo...");
    moverServoSuavemente(HOMBRO, 120);
    moverServoSuavemente(CODO, 70);


    // 3. Cerrar pinza (agarrar cubo)
    Serial.println("[Paso 3] Cerrando pinza...");
    moverServoSuavemente(PINZA, PINZA_CERRADA);
    delay(400); // Pausa de asentamiento

    // 4. Subir con el cubo y transportarlo
    Serial.println("[Paso 4] Transportando cubo...");
    moverServoSuavemente(HOMBRO, 70);
    moverServoSuavemente(CODO, 150);
    moverServoSuavemente(BASE, 30); // Mover hacia la zona de depósito

    // 5. Bajar en la zona de depósito
    Serial.println("[Paso 5] Bajando en zona de deposito...");
    moverServoSuavemente(HOMBRO, 120);
    moverServoSuavemente(CODO, 70);

    // 6. Soltar el cubo
    Serial.println("[Paso 6] Soltando cubo...");
    moverServoSuavemente(PINZA, PINZA_ABIERTA);
    delay(400);

    // 7. Retirarse y volver a inicio
    Serial.println("[Paso 7] Regresando a posicion inicial...");
    //moverServoSuavemente(HOMBRO, 70);
    volverAInicio();

    Serial.println("[INFO] === RUTINA FINALIZADA EXITOSAMENTE ===\n");
}

void volverAInicio() {
    Serial.println("[INFO] Volviendo a pose de inicio...");
    moverServoSuavemente(CODO, 90);
    moverServoSuavemente(HOMBRO, 90);
    moverServoSuavemente(BASE, ANGULO_INICIAL);
    moverServoSuavemente(PINZA, PINZA_ABIERTA);
}

// ==========================================
// CONTROL DE MOVIMIENTO INDIVIDUAL (PROBADO)
// ==========================================

/**
 * @brief Mueve un servo de forma gradual hasta el destino incluyendo yield() para la ESP32.
 * @param canal Canal del PCA9685 (0 a 3).
 * @param destino Ángulo final deseado (0 a 180 grados).
 */
void moverServoSuavemente(int canal, int destino) {
    int paso = (destino > anguloActual[canal]) ? 1 : -1;

    while (anguloActual[canal] != destino) {
        anguloActual[canal] += paso;
        posicionarServo(canal, anguloActual[canal]);
        delay(PAUSA_PASO_MS);
        
        // CRÍTICO EN ESP32: Evita que el Watchdog Timer reinicie la placa en movimientos largos
        yield(); 
    }
}

void posicionarServo(int canal, int angulo) {
    pca9685.setPWM(canal, 0, convertirAnguloAPwm(angulo));
}

int convertirAnguloAPwm(int angulo) {
    // Asegura que el ángulo esté estrictamente entre 0 y 180 antes de mapear
    int anguloRestringido = constrain(angulo, 0, 180);
    return map(anguloRestringido, 0, 180, PULSO_MINIMO, PULSO_MAXIMO);
}