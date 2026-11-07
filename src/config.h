/**
 * @file config.h
 * @brief Configuración central del brazo robótico: hardware, geometría D-H, calibración,
 *        puntos de trabajo, parámetros de movimiento y comunicación con la app.
 *
 * @details Todo valor que dependa del montaje físico o de la tarea se define en este archivo,
 *          de modo que el resto del programa no contiene constantes numéricas sueltas.
 *          Secciones:
 *            1. Pines y direcciones            4. Puntos conocidos
 *            2. Geometría (tabla D-H)          5. Parámetros de movimiento
 *            3. Calibración de servos          6. Comunicación con la app
 *
 * Convenciones del proyecto:
 *   - Longitudes en milímetros [mm], ángulos en grados [°] y tiempos en milisegundos [ms],
 *     salvo que el comentario de la constante indique otra unidad.
 *   - Sistema de coordenadas {0}: origen en el eje de giro de la base, X hacia adelante,
 *     Y hacia la izquierda y Z hacia arriba (Z = 0 sobre la superficie de la mesa).
 *   - Un valor igual a NAN se interpreta como "dato no definido" (ver definido()).
 *@warning se debe configurar WIFI_SSID y WIFI_CLAVE
 */
#pragma once
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include "driver_wifi.h"   // Aporta el objeto global Enlace (Serial + WiFi) que usa todo el programa

// ==========================================
// Pines y direcciones
// ==========================================

// Pines I2C estandar para la ESP32
constexpr int PIN_SDA = 21;   // GPIO de la línea SDA del bus I2C (pin por defecto de la ESP32)
constexpr int PIN_SCL = 22;   // GPIO de la línea SCL del bus I2C (pin por defecto de la ESP32)

// Direccion I2C del modulo PCA9685 (por defecto: 0x40)
constexpr uint8_t DIRECCION_PCA9685 = 0x40;   // Dirección I2C de 7 bits del PCA9685 (0x40 con los puentes de dirección sin soldar)

// Índice de cada servo. Coincide con el canal del PCA9685 y con la posición del servo en los
// arreglos de ángulos (poseActual.angulos, SERVO_OFFSET, LIMITE_SERVO_MIN, ...).
enum Articulacion {
    BASE = 0,   // Giro de la base alrededor del eje vertical
    HOMBRO = 1,   // Elevación del brazo
    CODO = 2,   // Flexión del antebrazo respecto del brazo
    PINZA = 3,   // Apertura y cierre de la garra (no interviene en la cinemática)
    CANTIDAD_SERVOS = 4   // Número de canales en uso; no es una articulación
};

// Articulaciones que intervienen en la cinemática: BASE, HOMBRO y CODO (la pinza no cuenta).
// Es el tamaño de los arreglos de calibración y de las soluciones de la cinemática inversa.
constexpr int CANTIDAD_ARTICULACIONES = 3;

// Sensor ultrasónico HC-SR04, montado en el brazo para detectar el cubo desde la pose de escaneo.
constexpr int TRIG_PIN = 18;   // GPIO de disparo (salida)
constexpr int ECHO_PIN = 33;   // GPIO del eco (entrada). El eco es de 5 V: requiere divisor de tensión hacia la ESP32 (3,3 V)
constexpr float UMBRAL_CM = 7.86f;   // [cm] Hay objeto si UMBRAL_CM >= distancia medida. Se fija con el comando 'd' como punto medio entre la lectura sin cubo y con cubo, medidas desde la pose de escaneo
constexpr int MUESTRAS_SENSOR = 5;   // Lecturas por medición; se informa la mediana, que descarta ecos espurios
constexpr int PAUSA_ENTRE_MUESTRAS_MS = 60;   // [ms] Separación entre disparos; evita que un eco tardío se confunda con el siguiente
constexpr unsigned long TIMEOUT_ECO_US = 30000;   // [µs] Espera máxima del eco (30 ms equivalen a unos 5 m de distancia)

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
constexpr float ALTURA_HOMBRO = 128.8f;   // d1 [mm]: altura del eje del hombro sobre la mesa
constexpr float LONGITUD_BRAZO = 71.8f;   // a2 [mm]: distancia entre el eje del hombro y el eje del codo
constexpr float LONGITUD_ANTEBRAZO = 51.5f;   // a3 [mm]: distancia entre el eje del codo y el inicio del tramo de la garra
constexpr float GARRA_DELTA_X = 87.0f;   // [mm] Componente del tramo de la garra a lo largo del antebrazo (hasta el punto de agarre)
constexpr float GARRA_DELTA_H = 13.0f;   // [mm] Desfase perpendicular del punto de agarre respecto de la línea del antebrazo (positivo = hacia abajo)

// Cubo de trabajo. El punto de agarre (origen del marco 4) es el centro del cubo.
constexpr int DIMENSION_CUBO = 20;   // [mm] Arista del cubo
constexpr float CUBO_Z_CENTRO = 10;   // [mm] Cota Z del centro del cubo apoyado en la mesa (DIMENSION_CUBO / 2). Debe actualizarse junto con DIMENSION_CUBO

// ==========================================
// Calibracion
// ==========================================

// Conversión de articulación (q, en grados) a ángulo de servo:
//     angulo_servo = SERVO_OFFSET + SERVO_SIGNO * SERVO_ESCALA * q
//
// Convención de q (la de la matriz T04 del Anexo II):
//   q1 (BASE):   0 = eje X, positivo en sentido antihorario visto desde arriba
//   q2 (HOMBRO): 0 = brazo horizontal hacia adelante, 90 = vertical, subir es positivo
//   q3 (CODO):   0 = antebrazo alineado con el brazo, negativo = codo flexionado hacia arriba
//
// Procedimiento: con el comando manual '<servo> <angulo>' se lleva cada articulación a la
// pose de referencia q = 0 y se anota el ángulo del servo (SERVO_OFFSET). El signo se obtiene
// observando si al aumentar el ángulo del servo aumenta o disminuye q. La escala es distinta de
// 1 solo si un mecanismo (palanca, paralelogramo) amplifica o reduce el giro del servo.
//
// Orden de los arreglos: BASE, HOMBRO, CODO.
constexpr float SERVO_OFFSET[CANTIDAD_ARTICULACIONES] = {90.0f, 70.0f, 77.0f};   // [°] Ángulo del servo cuando la articulación está en q = 0
constexpr float SERVO_SIGNO[CANTIDAD_ARTICULACIONES]  = {1, -1, 1};   // +1 o -1 según el sentido de montaje del servo
constexpr float SERVO_ESCALA[CANTIDAD_ARTICULACIONES] = {1.0f, 1.0f, 1.0f};   // 1.0 si el mecanismo es directo (relación 1:1)

// Rango permitido de cada servo en los movimientos con cinemática [°]. Debe elegirse con margen
// respecto de los topes mecánicos: un servo forzado contra un tope se sobrecalienta y se daña.
// Orden de los arreglos: BASE, HOMBRO, CODO.
constexpr float LIMITE_SERVO_MIN[CANTIDAD_ARTICULACIONES] = {0.0f,  10.0f,  1.0f};   
constexpr float LIMITE_SERVO_MAX[CANTIDAD_ARTICULACIONES] = {180.0f, 135.0f, 170.0f};  

// Señal PWM de los servos. El PCA9685 divide cada período en 4096 cuentas; a 50 Hz una cuenta
// dura unos 4,9 µs, por lo que 125 cuentas equivalen a ~0,6 ms y 575 a ~2,8 ms.
constexpr int PULSO_MINIMO = 125;   // Cuentas para 0°
constexpr int PULSO_MAXIMO = 575;   // Cuentas para 180°
constexpr int FRECUENCIA_PWM = 50;   // [Hz] Frecuencia de la señal para servos analógicos

// Ángulos de servo de la garra [°]. PINZA_CERRADA debe ser el ángulo en que las mordazas sujetan
// el cubo sin forzarlo; un valor mayor sobrecarga el servo y puede deformar el cubo.
constexpr int PINZA_ABIERTA = 26;   // Apertura suficiente para rodear el cubo
constexpr int PINZA_CERRADA = 86;   // Agarre del cubo

// Pose de inicio (home), expresada en ángulos de servo y no en coordenadas, de modo que es
// alcanzable aunque la calibración o la geometría estén mal definidas.
constexpr int ANGULO_INICIAL = 80;   // [°] Ángulo que se comanda a los cuatro servos al encender, antes de ir a HOME
constexpr int HOME_BASE = 70;   // [°] Base en la pose de reposo
constexpr int HOME_HOMBRO = 70;   // [°] Hombro en la pose de reposo
constexpr int HOME_CODO = 77;   // [°] Codo en la pose de reposo

// ==========================================
// Puntos conocidos
// ==========================================
// Coordenadas (X, Y) [mm] del CENTRO de cada cubo apoyado en la mesa. La cota Z se deduce de
// CUBO_Z_CENTRO. Se recomienda que todos los puntos estén a igual distancia de la base: así el
// desplazamiento entre ellos es un arco de radio constante y el brazo no se pliega de más
// (ver puntoEnTrayectoria()).
constexpr int CANTIDAD_POSICIONES = 3 ;   // Cubos a revisar; también es la altura máxima de la pila
constexpr float POSICION_X[CANTIDAD_POSICIONES] = {165.5f, 150.0f, 106.4f};   // Coordenada X de cada posición de recogida
constexpr float POSICION_Y[CANTIDAD_POSICIONES] = {  0.0f, 70.0f, 126.8f};   // Coordenada Y de cada posición de recogida

// Posición final: centro de la base de la pila. El primer cubo queda a nivel de la mesa y los
// siguientes encima, separados por DIMENSION_CUBO.
constexpr float PILA_X = 72.7f;   // [mm] Coordenada X de la pila
constexpr float PILA_Y = -155.9f;   // [mm] Coordenada Y de la pila

// ==========================================
// Parametros de movimiento
// ==========================================

// Velocidades. Cada movimiento se ejecuta en pasos pequeños separados por una pausa; la
// velocidad resultante es el tamaño del paso dividido por la pausa. Por ejemplo, en una
// trayectoria con PASO_TRAYECTORIA_MM = 2 mm y 15 ms de pausa la garra avanza ~133 mm/s.
constexpr int PAUSA_PASO_LIBRE_MS = 15;   // [ms] Pausa por paso de trayectoria sin carga
constexpr int PAUSA_PASO_TRANSPORTE_MS = 25;   // [ms] Pausa por paso de trayectoria con el cubo en la garra (más lenta)
constexpr int PAUSA_PASO_ARTICULAR_MS = 25;   // [ms] Pausa por grado en movimientos articulares (pinza y retorno a home)
constexpr int PAUSA_AGARRE_MS = 400;   // [ms] Espera tras abrir o cerrar la pinza, para que el agarre se asiente
constexpr int PAUSA_ESCANEO_MS = 300;   // [ms] Espera a que el brazo deje de vibrar antes de medir con el sensor

// Discretización de las trayectorias.
constexpr float PASO_TRAYECTORIA_MM = 2.0f;   // [mm] Distancia entre puntos intermedios de una trayectoria
constexpr float MAX_SALTO_ARTICULAR_GRADOS = 10.0f;   // [°] Cambio máximo de un servo entre dos pasos; detecta cambios bruscos de configuración de la cinemática inversa

// Alturas de trabajo (distancias verticales respecto del punto de agarre).
constexpr float ALTURA_APROXIMACION_MM = 22.0f;   // [mm] Altura sobre el centro del cubo de la pose de aproximación y de escaneo. Debe librar la cima del cubo y las mordazas. Si se modifica, hay que recalibrar UMBRAL_CM
constexpr float ALTURA_APROXIMACION_PILA_MM = 55.0f;   // [mm] Altura sobre el punto de depósito de cada nivel a la que se transita sobre la pila; debe librar la cima de la pila con el cubo en la garra
constexpr float MARGEN_DEPOSITO_MM = 5.0f;   // [mm] Holgura vertical al soltar, para no aplastar el cubo contra la pila

// Posición del sensor respecto del punto de agarre, medida en la dirección radial desde la base.
// Positivo = el sensor queda por delante de la garra, por lo que la pose de escaneo retrocede esa
// distancia hacia la base; negativo = el sensor queda por detrás y la pose avanza.
constexpr float DESFASE_SENSOR_MM = -20.0f;   // [mm]

// Enlace con la app (vista en el PC o en el celular). Los mensajes salen por USB y por WiFi.
constexpr unsigned long BAUDIOS_SERIE = 115200;   // [baudios] Velocidad del puerto USB; debe coincidir con la del Monitor Serie y la vista
constexpr unsigned long TELEMETRIA_MS = 1000;   // [ms] Período de emisión de los mensajes STAT, DH y RUT hacia la app

// WiFi: la app se conecta por TCP a <IP del robot>:WIFI_PUERTO.
// Se intenta primero la red WIFI_SSID; si no se logra conectar en WIFI_TIMEOUT_MS, el robot crea su
// propia red (punto de acceso AP_SSID, IP 192.168.4.1) para poder usarse sin infraestructura.
constexpr char WIFI_SSID[]  = "";   // Red existente a la que se conecta el robot (credencial: ver @warning de la cabecera)
constexpr char WIFI_CLAVE[] = "";   // Clave de la red existente (credencial: ver @warning de la cabecera)
constexpr char AP_SSID[]    = "BrazoRobot";   // Nombre de la red propia de respaldo
constexpr char AP_CLAVE[]   = "12345678";   // Clave de la red propia (mínimo 8 caracteres)
constexpr uint16_t WIFI_PUERTO = 8080;   // Puerto TCP del servidor del robot
constexpr unsigned long WIFI_TIMEOUT_MS = 10000;   // [ms] Espera máxima para conectarse a la red antes de crear el punto de acceso

// Rango permitido para el control manual desde la app (comando S,<canal>,<angulo>): BASE,
// HOMBRO y CODO usan LIMITE_SERVO_MIN/MAX; la pinza, el intervalo entre abierta y cerrada.
constexpr int LIMITE_PINZA_MIN = (PINZA_ABIERTA < PINZA_CERRADA) ? PINZA_ABIERTA : PINZA_CERRADA;
constexpr int LIMITE_PINZA_MAX = (PINZA_ABIERTA < PINZA_CERRADA) ? PINZA_CERRADA : PINZA_ABIERTA;

