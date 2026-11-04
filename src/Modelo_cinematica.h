// ##########################################################################
// >>> ARCHIVO 2: Modelo_Cinematica (.h y .cpp)
// ##########################################################################
#pragma once
#include "config.h"
#include <math.h>
// Tipos de datos. Van antes de cualquier funcion para que el IDE de Arduino
// los conozca al generar los prototipos.

struct Punto {
    float x;
    float y;
    float z;
};

struct Matriz4 {
    float m[4][4];
};

// Resultado de calcular los angulos de servo para un punto
enum EstadoIK {
    IK_OK = 0,
    IK_FUERA_DE_ALCANCE,
    IK_SIN_CALIBRACION,
    IK_FUERA_DE_LIMITES
};

// Parametros derivados del tramo de la garra (fila 4 de la tabla D-H)
static const float GARRA_A4  = sqrtf(GARRA_DELTA_X * GARRA_DELTA_X + GARRA_DELTA_H * GARRA_DELTA_H);
static const float GARRA_PHI = atan2f(GARRA_DELTA_H, GARRA_DELTA_X);   // radianes

inline bool definido(float valor) {
    return !isnan(valor);
}

// >>> MODELO_CINEMATICA_INICIO
float normalizar180(float grados) {
    while (grados > 180.0f) grados -= 360.0f;
    while (grados <= -180.0f) grados += 360.0f;
    return grados;
}

/**
 * @brief Matriz de transformacion D-H estandar: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha).
 */
Matriz4 matrizDH(float thetaRad, float d, float a, float alphaRad) {
    const float ct = cosf(thetaRad);
    const float st = sinf(thetaRad);
    const float ca = cosf(alphaRad);
    const float sa = sinf(alphaRad);
    Matriz4 A = {{
        {ct, -st * ca,  st * sa, a * ct},
        {st,  ct * ca, -ct * sa, a * st},
        {0.0f, sa,      ca,      d},
        {0.0f, 0.0f,    0.0f,    1.0f}
    }};
    return A;
}

Matriz4 multiplicar(const Matriz4& A, const Matriz4& B) {
    Matriz4 C;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float suma = 0.0f;
            for (int k = 0; k < 4; k++) {
                suma += A.m[i][k] * B.m[k][j];
            }
            C.m[i][j] = suma;
        }
    }
    return C;
}

/**
 * @brief Cinematica directa por D-H. Entrega las transformaciones acumuladas:
 *        T[0] = base, T[1] = hombro, T[2] = codo, T[3] = muneca, T[4] = garra (T04 completa).
 * @param qGrados Articulaciones [q1, q2, q3] en grados.
 */
void cadenaDH(const float qGrados[], Matriz4 T[5]) {
    const Matriz4 identidad = {{
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    }};
    const Matriz4 A1 = matrizDH(qGrados[BASE] * DEG_TO_RAD,   ALTURA_HOMBRO, 0.0f,               90.0f * DEG_TO_RAD);
    const Matriz4 A2 = matrizDH(qGrados[HOMBRO] * DEG_TO_RAD, 0.0f,          LONGITUD_BRAZO,     0.0f);
    const Matriz4 A3 = matrizDH(qGrados[CODO] * DEG_TO_RAD,   0.0f,          LONGITUD_ANTEBRAZO, 0.0f);
    const Matriz4 A4 = matrizDH(-GARRA_PHI,                   0.0f,          GARRA_A4,           0.0f);

    T[0] = identidad;
    T[1] = A1;
    T[2] = multiplicar(T[1], A2);
    T[3] = multiplicar(T[2], A3);
    T[4] = multiplicar(T[3], A4);
}

Matriz4 cinematicaDirecta(const float qGrados[]) {
    Matriz4 T[5];
    cadenaDH(qGrados, T);
    return T[4];
}

Punto posicionDe(const Matriz4& T) {
    Punto p = {T.m[0][3], T.m[1][3], T.m[2][3]};
    return p;
}

/**
 * @brief Cinematica inversa analitica (codo arriba) del punto de agarre (x, y, z).
 *
 * Usa las definiciones de la imagen: l = sqrt(x^2 + y^2), h = sqrt(l^2 + z^2),
 * alfa = atan(z / l), beta = acos((L2^2 + h^2 - L3^2) / (2 L2 h)), q1 = atan(y / x),
 * con z medida desde el eje del hombro, L2 = LONGITUD_BRAZO y L3 = distancia del eje del
 * codo al punto de agarre (sqrt((a3 + dx)^2 + dh^2)).
 *
 * DIFERENCIA CON LA IMAGEN: alli q2 = 90 - (alfa + beta) y q3 = alfa + beta. Con la matriz T04
 * (R = a2 c2 + (a3+dx) c23 + dh s23), q2 = alfa + beta es la elevacion del brazo, y q3 es el
 * giro relativo del antebrazo, que NO es alfa + beta (daria siempre q3 = 90 - q2). Aqui q3 se
 * obtiene de la direccion codo -> garra. Se comprueba en autopruebaCinematica().
 *
 * @param objetivo Punto de agarre en mm.
 * @param qGrados  Salida [q1, q2, q3] en grados.
 * @return false si el punto esta fuera del alcance.
 */
bool cinematicaInversa(const Punto& objetivo, float qGrados[]) {
    // Vector codo -> garra visto desde el antebrazo: (a3 + dx, -dh)
    const float ax = LONGITUD_ANTEBRAZO + GARRA_DELTA_X;
    const float ay = -GARRA_DELTA_H;
    const float L2 = LONGITUD_BRAZO;
    const float L3 = sqrtf(ax * ax + ay * ay);
    const float delta = atan2f(ay, ax);

    const float q1 = atan2f(objetivo.y, objetivo.x);
    const float l = sqrtf(objetivo.x * objetivo.x + objetivo.y * objetivo.y);
    const float zr = objetivo.z - ALTURA_HOMBRO;
    const float h = sqrtf(l * l + zr * zr);
    if (h < 1e-3f) return false;

    const float alfa = atan2f(zr, l);
    const float cosBeta = (L2 * L2 + h * h - L3 * L3) / (2.0f * L2 * h);
    if (cosBeta < -1.0f || cosBeta > 1.0f) return false;   // punto fuera del alcance
    const float beta = acosf(cosBeta);

    const float q2 = alfa + beta;                                              // codo arriba
    const float psi = atan2f(zr - L2 * sinf(q2), l - L2 * cosf(q2));           // direccion codo -> garra
    const float q3 = psi - q2 - delta;

    qGrados[BASE] = normalizar180(q1 * RAD_TO_DEG);
    qGrados[HOMBRO] = normalizar180(q2 * RAD_TO_DEG);
    qGrados[CODO] = normalizar180(q3 * RAD_TO_DEG);
    return true;
}

// ---- Conversion articulacion <-> servo (calibracion) ----

bool articulacionAServo(int j, float qGrados, float& servoGrados) {
    if (!definido(SERVO_OFFSET[j]) || !definido(SERVO_SIGNO[j])) return false;
    servoGrados = SERVO_OFFSET[j] + SERVO_SIGNO[j] * SERVO_ESCALA[j] * qGrados;
    return true;
}

float servoAArticulacion(int j, float servoGrados) {
    return (servoGrados - SERVO_OFFSET[j]) / (SERVO_SIGNO[j] * SERVO_ESCALA[j]);
}

/**
 * @brief Punto -> angulos de servo de BASE, HOMBRO y CODO (con comprobacion de alcance y limites).
 */
EstadoIK calcularAngulosServo(const Punto& punto, float servo[]) {
    float q[CANTIDAD_ARTICULACIONES];
    if (!cinematicaInversa(punto, q)) return IK_FUERA_DE_ALCANCE;

    for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
        if (!articulacionAServo(j, q[j], servo[j])) return IK_SIN_CALIBRACION;
    }
    for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
        if (servo[j] < LIMITE_SERVO_MIN[j] || servo[j] > LIMITE_SERVO_MAX[j]) return IK_FUERA_DE_LIMITES;
    }
    return IK_OK;
}

const char* describirEstadoIK(EstadoIK estado) {
    switch (estado) {
        case IK_OK:                return "ok";
        case IK_FUERA_DE_ALCANCE:  return "fuera de alcance";
        case IK_SIN_CALIBRACION:   return "falta calibracion (SERVO_OFFSET / SERVO_SIGNO)";
        case IK_FUERA_DE_LIMITES:  return "fuera de los limites de servo";
    }
    return "desconocido";
}
// <<< MODELO_CINEMATICA_FIN