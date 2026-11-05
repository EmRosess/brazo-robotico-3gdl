/**CONFIGURACIÓN DE CONSTANTES
*/
#pragma once
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

// Valor que marca un dato que aun no se ha medido o definido.
// El programa no mueve el brazo con cinematica mientras quede alguno sin completar.
//constexpr float NO_DEFINIDO = NAN;

// ==========================================
// Pines y direcciones
// ==========================================

// Pines I2C estandar para la ESP32
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// Direccion I2C del modulo PCA9685 (por defecto: 0x40)
constexpr uint8_t DIRECCION_PCA9685 = 0x40;

// Asignacion de canales del PCA9685 a cada articulacion
enum Articulacion {
    BASE = 0,
    HOMBRO = 1,
    CODO = 2,
    PINZA = 3,
    CANTIDAD_SERVOS = 4
};

// Articulaciones que participan en la cinematica (la pinza no cuenta)
constexpr int CANTIDAD_ARTICULACIONES = 3;

// Pines del sensor HC-SR04 y umbral
constexpr int TRIG_PIN = 18;
constexpr int ECHO_PIN = 33;                      // ECHO entrega 5 V: usar divisor de tension hacia la ESP32
constexpr float UMBRAL_CM = 10.0f;                 // AJUSTAR: si UMBRAL_CM >= distancia medida, hay objeto (calibrar con el comando 'd')
constexpr int MUESTRAS_SENSOR = 5;                // Lecturas por medicion (se usa la mediana)
constexpr int PAUSA_ENTRE_MUESTRAS_MS = 60;
constexpr unsigned long TIMEOUT_ECO_US = 30000;   // ~5 m

// ==========================================
// Geometria (tabla D-H)
// ==========================================

// Sistema de coordenadas (mm): origen en el eje de giro de la base,
// X hacia adelante, Y hacia la izquierda, Z hacia arriba (Z = 0 en la mesa).
//
//   fila | theta  | d              | a                      | alpha
//    1   | q1     | ALTURA_HOMBRO  | 0                      | 90 deg
//    2   | q2     | 0              | LONGITUD_BRAZO         | 0
//    3   | q3     | 0              | LONGITUD_ANTEBRAZO     | 0
//    4   | -phi   | 0              | a4 (tramo de la garra) | 0
//
// con a4 = sqrt(GARRA_DELTA_X^2 + GARRA_DELTA_H^2) y phi = atan2(GARRA_DELTA_H, GARRA_DELTA_X)
constexpr float ALTURA_HOMBRO = 129.5f;         // d1: eje del hombro sobre la mesa
constexpr float LONGITUD_BRAZO = 70.53f;        // a2: eje del hombro -> eje del codo
constexpr float LONGITUD_ANTEBRAZO = 50.035f;   // a3: eje del codo -> inicio del tramo de la garra
constexpr float GARRA_DELTA_X = 100.3f;         // tramo de la garra, a lo largo del antebrazo
constexpr float GARRA_DELTA_H = 15.0f;           // el punto de agarre queda esto por debajo de la linea del antebrazo
                                                // AJUSTAR: a3 y GARRA_DELTA_X se dedujeron de a3 + GARRA_DELTA_X = 139,645 mm

// tamano del cubo en mm (el punto de agarre es el centro del cubo)
constexpr int DIMENSION_CUBO = 15;
constexpr float CUBO_Z_CENTRO = 7.5;   // centro del cubo apoyado en la mesa

// ==========================================
// Calibracion
// ==========================================

// Conversion de articulacion (q, en grados) a angulo de servo:
//     angulo_servo = SERVO_OFFSET + SERVO_SIGNO * SERVO_ESCALA * q
//
// Convencion de q (la de la matriz T04):
//   q1 (BASE):   0 = eje X, positivo en sentido antihorario visto desde arriba
//   q2 (HOMBRO): 0 = brazo horizontal hacia adelante, 90 = vertical, subir es positivo
//   q3 (CODO):   0 = antebrazo alineado con el brazo, negativo = codo flexionado hacia arriba
//
// orden: BASE, HOMBRO, CODO
constexpr float SERVO_OFFSET[CANTIDAD_ARTICULACIONES] = {90.0f, 70.0f, 77.0f};  // PENDIENTE: angulo del servo cuando q = 0
constexpr float SERVO_SIGNO[CANTIDAD_ARTICULACIONES]  = {1, -1, 1};  // PENDIENTE: +1 o -1 segun el montaje
constexpr float SERVO_ESCALA[CANTIDAD_ARTICULACIONES] = {1.0f, 1.0f, 1.0f};                       // AJUSTAR: 1.0 si el mecanismo es directo (1:1)

// rango de movimiento de servos (solo para movimientos con cinematica)
// orden: BASE, HOMBRO, CODO. Valores iniciales = rango de la rutina con angulos fijos ya probada.
constexpr float LIMITE_SERVO_MIN[CANTIDAD_ARTICULACIONES] = {0.0f,  10.0f,  1.0f};   // AJUSTAR: amplia sin forzar los topes
constexpr float LIMITE_SERVO_MAX[CANTIDAD_ARTICULACIONES] = {180.0f, 135.0f, 170.0f};  // AJUSTAR: amplia sin forzar los topes

// Configuracion de pulsos PWM para servomotores
constexpr int PULSO_MINIMO = 125;     // Equivalente aprox. a 0 grados (~0,6 ms con oscilador de 25 MHz)
constexpr int PULSO_MAXIMO = 575;     // Equivalente aprox. a 180 grados (~2,8 ms con oscilador de 25 MHz)
constexpr int FRECUENCIA_PWM = 50;    // Frecuencia en Hz para servos

// angulo de pinza abierta y cerrada (grados)
constexpr int PINZA_ABIERTA = 50;     // AJUSTAR: 
constexpr int PINZA_CERRADA = 70;     // AJUSTAR: angulo en que las mordazas sujetan el cubo sin forzarlo

// Pose de inicio (home): angulos de servo (tomados de la rutina con angulos fijos ya probada)
constexpr int ANGULO_INICIAL =80;    // Todos los servos al encender
constexpr int HOME_BASE = 70;
constexpr int HOME_HOMBRO = 70;
constexpr int HOME_CODO = 77;

// ==========================================
// Puntos conocidos
// ==========================================
// Coordenadas (X, Y) en mm del CENTRO de cada cubo apoyado en la mesa.
// La altura (Z) se calcula sola a partir de DIMENSION_CUBO.
// Las posiciones son equidistantes: usa la misma separacion entre una y otra.

constexpr int CANTIDAD_POSICIONES = 3 ;
constexpr float POSICION_X[CANTIDAD_POSICIONES] = {165.5f, 150.0f, 106.4f}; //150,70
constexpr float POSICION_Y[CANTIDAD_POSICIONES] = {  0.0f, 70.0f, 126.8f};

// posicion final: centro de la base de la pila (el 1er cubo queda aqui, los demas encima)
constexpr float PILA_X = 28.7f;  // PENDIENTE 140,100
constexpr float PILA_Y = -163.0f;  // PENDIENTE

// ==========================================
// Parametros de movimiento
// ==========================================

// Velocidades (pausa en ms por cada paso)
constexpr int PAUSA_PASO_LIBRE_MS = 15;        // Trayectoria sin carga (cada PASO_TRAYECTORIA_MM)
constexpr int PAUSA_PASO_TRANSPORTE_MS = 25;   // Trayectoria transportando el cubo
constexpr int PAUSA_PASO_ARTICULAR_MS = 25;    // Movimiento por articulacion (pinza, home): 1 grado por paso
constexpr int PAUSA_AGARRE_MS = 400;           // Espera tras abrir/cerrar la pinza
constexpr int PAUSA_ESCANEO_MS = 300;          // Espera a que el brazo deje de vibrar antes de medir

// Trayectoria en linea recta
constexpr float PASO_TRAYECTORIA_MM = 2.0f;           // Distancia entre puntos intermedios
constexpr float MAX_SALTO_ARTICULAR_GRADOS = 10.0f;   // Cambio maximo de un servo entre dos pasos (evita cambios bruscos de configuracion)

// Alturas
constexpr float ALTURA_APROXIMACION_MM = 15.0f;   // AJUSTAR: sobre el punto de agarre; debe librar la cima del cubo y las mordazas
constexpr float ALTURA_APROXIMACION_PILA_MM = 10.0f;   // AJUSTAR: sobre el punto de deposito de cada nivel; debe librar la cima de la pila con el cubo en la garra
constexpr float MARGEN_DEPOSITO_MM = 2.0f;        // Holgura al soltar, para no aplastar el cubo

// Sensor montado en el brazo: cuanto queda el sensor por delante del punto de agarre, en mm
// (positivo = adelante). La pose de escaneo retrocede esa distancia para dejar el sensor sobre el cubo.
constexpr float DESFASE_SENSOR_MM = 0.0f;         // AJUSTAR
