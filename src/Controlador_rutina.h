

// ##########################################################################
// >>> ARCHIVO 7: Controlador_Rutina (.h y .cpp)
// ##########################################################################
#pragma once
#include "Driver_servos.h"
#include "Driver_sensor.h"
#include "Modelo_cinematica.h"
#include "Controlador_comandos.h"
#include "Modelo_brazo.h"
#include "Vista_telemetria.h"
// ---- Puntos de la rutina (a partir de la configuracion) ----

Punto puntoRecogida(int n) {
    Punto p = {POSICION_X[n], POSICION_Y[n], CUBO_Z_CENTRO};
    return p;
}

Punto puntoAproximacion(int n) {
    Punto p = {POSICION_X[n], POSICION_Y[n], CUBO_Z_CENTRO + ALTURA_APROXIMACION_MM};
    return p;
}

// Pose de escaneo: sobre el cubo, retrocediendo DESFASE_SENSOR_MM para que el sensor quede encima
Punto puntoEscaneo(int n) {
    Punto p = puntoAproximacion(n);
    float l = sqrtf(p.x * p.x + p.y * p.y);
    if (l > 1e-3f) {
        p.x -= DESFASE_SENSOR_MM * p.x / l;
        p.y -= DESFASE_SENSOR_MM * p.y / l;
    }
    return p;
}

// Sobre la pila: nunca mas bajo que la altura de aproximacion de recogida, asi el cubo transportado
// no roza los cubos que quedan en la mesa (el traslado sube o se mantiene, no baja).
Punto puntoSobrePila(int nivel) {
    float zNivel = CUBO_Z_CENTRO + nivel * DIMENSION_CUBO + ALTURA_APROXIMACION_PILA_MM;
    float zTransito = CUBO_Z_CENTRO + ALTURA_APROXIMACION_MM;
    Punto p = {PILA_X, PILA_Y, fmaxf(zNivel, zTransito)};
    return p;
}

Punto puntoDeposito(int nivel) {
    Punto p = {PILA_X, PILA_Y, CUBO_Z_CENTRO + nivel * DIMENSION_CUBO + MARGEN_DEPOSITO_MM};
    return p;
}

/**
 * @brief Hay objeto si el umbral es mayor o igual que la distancia medida.
 */
bool hayObjeto(float distanciaCm) {
    return distanciaCm > 0.0f && UMBRAL_CM >= distanciaCm;
}

// ---- Trayectorias ----

float radioDe(const Punto& p) {
    return sqrtf(p.x * p.x + p.y * p.y);
}

// Giro de la base (radianes, por el camino mas corto) para ir de a hasta b
float giroBase(const Punto& a, const Punto& b) {
    if (radioDe(a) < 1e-3f || radioDe(b) < 1e-3f) return 0.0f;   // un punto sobre el eje no define angulo
    float dt = atan2f(b.y, b.x) - atan2f(a.y, a.x);
    while (dt > PI) dt -= 2.0f * PI;
    while (dt < -PI) dt += 2.0f * PI;
    return dt;
}

/**
 * @brief Punto de la trayectoria de a hasta b para la fraccion f (0 a 1).
 *        Se interpolan por separado el angulo de la base, el radio horizontal y la altura:
 *        - entre dos puntos con el mismo radio la garra recorre un ARCO: solo gira la base
 *          y el brazo no se pliega mas que en los extremos;
 *        - en una subida o bajada vertical recorre una recta vertical.
 *        Una recta entre dos puntos de un mismo circulo cortaria por dentro del circulo y
 *        obligaria al brazo a plegarse mucho mas que en los extremos.
 */
Punto puntoEnTrayectoria(const Punto& a, const Punto& b, float f) {
    const float la = radioDe(a);
    const float lb = radioDe(b);
    float ta = atan2f(a.y, a.x);
    if (la < 1e-3f) ta = atan2f(b.y, b.x);

    const float l = la + f * (lb - la);
    const float t = ta + f * giroBase(a, b);
    Punto p = {l * cosf(t), l * sinf(t), a.z + f * (b.z - a.z)};
    return p;
}

// Longitud aproximada de la trayectoria (mm): arco medio + variacion de radio + variacion de altura
float longitudTrayectoria(const Punto& a, const Punto& b) {
    const float dl = radioDe(b) - radioDe(a);
    const float arco = 0.5f * (radioDe(a) + radioDe(b)) * fabsf(giroBase(a, b));
    const float dz = b.z - a.z;
    return sqrtf(dl * dl + arco * arco + dz * dz);
}

// ---- Verificaciones previas ----

/**
 * @brief Revisa que no falte ningun dato de la configuracion e informa cuales faltan.
 */
bool configuracionLista() {
    const char* nombres[CANTIDAD_ARTICULACIONES] = {"BASE", "HOMBRO", "CODO"};
    int faltantes = 0;

    for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
        if (!definido(SERVO_OFFSET[j])) {
            Enlace.print("  - SERVO_OFFSET de ");
            Enlace.println(nombres[j]);
            faltantes++;
        }
        if (!definido(SERVO_SIGNO[j])) {
            Enlace.print("  - SERVO_SIGNO de ");
            Enlace.println(nombres[j]);
            faltantes++;
        }
    }
    for (int n = 0; n < CANTIDAD_POSICIONES; n++) {
        if (!definido(POSICION_X[n]) || !definido(POSICION_Y[n])) {
            Enlace.print("  - POSICION_X / POSICION_Y del cubo ");
            Enlace.println(n + 1);
            faltantes++;
        }
    }
    if (!definido(PILA_X) || !definido(PILA_Y)) {
        Enlace.println("  - PILA_X / PILA_Y (posicion final)");
        faltantes++;
    }

    if (faltantes > 0) {
        Enlace.print("[AVISO] Faltan ");
        Enlace.print(faltantes);
        Enlace.println(" dato(s) en la configuracion (arriba). La rutina no se ejecutara hasta completarlos.");
        return false;
    }
    return true;
}

bool verificarPunto(const char* nombre, int indice, const Punto& p) {
    float servo[CANTIDAD_ARTICULACIONES];
    EstadoIK estado = calcularAngulosServo(p, servo);
    if (estado == IK_OK) return true;

    Enlace.print("[ERROR] ");
    Enlace.print(nombre);
    Enlace.print(" ");
    Enlace.print(indice);
    Enlace.print(": ");
    Enlace.println(describirEstadoIK(estado));
    mostrarPunto("        punto", p);
    return false;
}

/**
 * @brief Comprueba que todo el tramo entre a y b (por la misma trayectoria del movimiento) es alcanzable.
 */
bool tramoAlcanzable(const Punto& a, const Punto& b, const char* nombre) {
    int MUESTRAS = (int)ceilf(longitudTrayectoria(a, b) / 5.0f);   // una muestra cada 5 mm como maximo
    if (MUESTRAS < 20) MUESTRAS = 20;
    float servo[CANTIDAD_ARTICULACIONES];
    for (int i = 0; i <= MUESTRAS; i++) {
        float f = (float)i / MUESTRAS;
        Punto p = puntoEnTrayectoria(a, b, f);   // la misma trayectoria que usa el movimiento
        EstadoIK estado = calcularAngulosServo(p, servo);
        if (estado != IK_OK) {
            Enlace.print("[ERROR] Tramo ");
            Enlace.print(nombre);
            Enlace.print(": ");
            Enlace.println(describirEstadoIK(estado));
            mostrarPunto("        punto", p);
            return false;
        }
    }
    return true;
}

/**
 * @brief Revisa antes de mover que todos los puntos y tramos de la rutina son alcanzables
 *        y quedan dentro de los limites de servo.
 */
bool validarRutina() {
    bool ok = true;

    for (int n = 0; n < CANTIDAD_POSICIONES; n++) {
        if (!verificarPunto("Pose de escaneo", n + 1, puntoEscaneo(n))) ok = false;
        if (!verificarPunto("Aproximacion", n + 1, puntoAproximacion(n))) ok = false;
        if (!verificarPunto("Recogida", n + 1, puntoRecogida(n))) ok = false;
        if (!tramoAlcanzable(puntoAproximacion(n), puntoRecogida(n), "bajar a recoger")) ok = false;
        if (!tramoAlcanzable(puntoEscaneo(n), puntoAproximacion(n), "escaneo -> aproximacion")) ok = false;
    }
    for (int k = 0; k < CANTIDAD_POSICIONES; k++) {
        if (!verificarPunto("Sobre la pila, nivel", k + 1, puntoSobrePila(k))) ok = false;
        if (!verificarPunto("Deposito, nivel", k + 1, puntoDeposito(k))) ok = false;
        if (!tramoAlcanzable(puntoSobrePila(k), puntoDeposito(k), "bajar a depositar")) ok = false;
        for (int n = 0; n < CANTIDAD_POSICIONES; n++) {
            if (!tramoAlcanzable(puntoAproximacion(n), puntoSobrePila(k), "recogida -> pila")) ok = false;
            if (!tramoAlcanzable(puntoSobrePila(k), puntoEscaneo(n), "pila -> siguiente posicion")) ok = false;
        }
    }

    if (ok) {
        Enlace.println("[OK] Todos los puntos y tramos de la rutina son alcanzables.");
    }
    return ok;
}

/**
 * @brief Comprueba que la inversa devuelve el mismo punto que produce la directa.
 */
void autopruebaCinematica() {
    const float pruebas[4][CANTIDAD_ARTICULACIONES] = {
        {0.0f, 20.0f, -40.0f},
        {20.0f, 30.0f, -50.0f},
        {-25.0f, 10.0f, -70.0f},
        {40.0f, 45.0f, -30.0f}
    };

    float errorMax = 0.0f;
    bool todoOk = true;
    for (int i = 0; i < 4; i++) {
        Punto objetivo = posicionDe(cinematicaDirecta(pruebas[i]));
        float q[CANTIDAD_ARTICULACIONES];
        if (!cinematicaInversa(objetivo, q)) {
            todoOk = false;
            continue;
        }
        Punto obtenido = posicionDe(cinematicaDirecta(q));
        float dx = obtenido.x - objetivo.x;
        float dy = obtenido.y - objetivo.y;
        float dz = obtenido.z - objetivo.z;
        float error = sqrtf(dx * dx + dy * dy + dz * dz);
        if (error > errorMax) errorMax = error;
    }

    if (todoOk && errorMax < 0.01f) {
        Enlace.print("[OK] Autoprueba de cinematica (IK -> FK): error maximo ");
        Enlace.print(errorMax, 4);
        Enlace.println(" mm");
    } else {
        Enlace.println("[ERROR] Autoprueba de cinematica fallida: revisa la tabla D-H y la IK.");
    }
}

// ---- Movimientos ----

/**
 * @brief Mueve la garra desde su posicion actual hasta el destino siguiendo puntoEnTrayectoria()
 *        (arco a radio constante entre puntos del mismo radio, vertical al subir y bajar).
 *        Cada punto intermedio se convierte en angulos con la cinematica inversa.
 */
bool moverEnTrayectoria(const Punto& destino, int pausaMs) {
    if (detenerRutina) return false;

    Punto inicio;
    if (!obtenerPuntoActual(inicio)) {
        Enlace.println("[ERROR] Sin calibracion: no se puede calcular la posicion actual.");
        return false;
    }

    int pasos = (int)ceilf(longitudTrayectoria(inicio, destino) / PASO_TRAYECTORIA_MM);
    if (pasos < 1) pasos = 1;

    for (int i = 1; i <= pasos; i++) {
        if (paradaSolicitada()) return false;

        float t = (1.0f - cosf(PI * i / pasos)) / 2.0f;   // arranque y frenado suaves
        Punto p = puntoEnTrayectoria(inicio, destino, t);

        float servo[CANTIDAD_ARTICULACIONES];
        EstadoIK estado = calcularAngulosServo(p, servo);
        if (estado != IK_OK) {
            Enlace.print("[ERROR] Movimiento detenido: ");
            Enlace.println(describirEstadoIK(estado));
            mostrarPunto("        punto", p);
            return false;
        }

        for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
            if (fabsf(servo[j] - poseActual.angulos[j]) > MAX_SALTO_ARTICULAR_GRADOS) {
                Enlace.println("[ERROR] Movimiento detenido: salto articular excesivo (cambio de configuracion).");
                mostrarPunto("        punto", p);
                return false;
            }
        }

        for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
            poseActual.angulos[j] = servo[j];
            posicionarServo(j, servo[j]);
        }
        delay(pausaMs);
        yield();
    }
    return true;
}

/**
 * @brief Lleva la garra al punto moviendo las tres articulaciones a la vez (movimiento articular).
 *        Se usa para el primer desplazamiento desde home y para el comando 'g'.
 */
bool moverAPuntoArticular(const Punto& destino) {
    float servo[CANTIDAD_ARTICULACIONES];
    EstadoIK estado = calcularAngulosServo(destino, servo);
    if (estado != IK_OK) {
        Enlace.print("[ERROR] No se puede ir al punto: ");
        Enlace.println(describirEstadoIK(estado));
        mostrarPunto("        punto", destino);
        return false;
    }

    float objetivo[CANTIDAD_SERVOS];
    for (int j = 0; j < CANTIDAD_ARTICULACIONES; j++) {
        objetivo[j] = servo[j];
    }
    objetivo[PINZA] = poseActual.angulos[PINZA];

    moverServosSincronizados(objetivo, PAUSA_PASO_ARTICULAR_MS);
    return !detenerRutina;
}

void volverAInicio() {
    Enlace.println("[INFO] Volviendo a pose de inicio...");
    moverServoSuavemente(CODO, HOME_CODO, PAUSA_PASO_ARTICULAR_MS);
    moverServoSuavemente(HOMBRO, HOME_HOMBRO, PAUSA_PASO_ARTICULAR_MS);
    moverServoSuavemente(BASE, HOME_BASE, PAUSA_PASO_ARTICULAR_MS);
    moverServoSuavemente(PINZA, PINZA_ABIERTA, PAUSA_PASO_ARTICULAR_MS);
}

/**
 * @brief Baja, toma el cubo, sube, lo lleva a la posicion final y lo deja en el nivel indicado.
 * @param posicion Indice de la posicion de recogida (0 a CANTIDAD_POSICIONES - 1).
 * @param nivel    Cubos ya apilados (0 = primer nivel). Define la altura de deposito.
 */
bool transportarObjeto(int posicion, int nivel) {
    nivel = constrain(nivel, 0, CANTIDAD_POSICIONES - 1);

    Enlace.print("[INFO] Transportando a la pila (nivel ");
    Enlace.print(nivel + 1);
    Enlace.println(")");

    // 1. Alinear la garra sobre el cubo y bajar
    if (!moverEnTrayectoria(puntoAproximacion(posicion), PAUSA_PASO_LIBRE_MS)) return false;
    if (!moverEnTrayectoria(puntoRecogida(posicion), PAUSA_PASO_LIBRE_MS)) return false;

    // 2. Tomar (cerrar pinza)
    moverServoSuavemente(PINZA, PINZA_CERRADA, PAUSA_PASO_ARTICULAR_MS);
    esperarMs(PAUSA_AGARRE_MS);

    // 3. Subir con el cubo
    if (!moverEnTrayectoria(puntoAproximacion(posicion), PAUSA_PASO_TRANSPORTE_MS)) return false;

    // 4. Mover a la posicion final (sobre la pila)
    if (!moverEnTrayectoria(puntoSobrePila(nivel), PAUSA_PASO_TRANSPORTE_MS)) return false;

    // 5. Bajar sobre la pila (la altura depende del nivel)
    if (!moverEnTrayectoria(puntoDeposito(nivel), PAUSA_PASO_TRANSPORTE_MS)) return false;

    // 6. Soltar el cubo
    moverServoSuavemente(PINZA, PINZA_ABIERTA, PAUSA_PASO_ARTICULAR_MS);
    esperarMs(PAUSA_AGARRE_MS);

    // 7. Subir libre de la pila
    if (!moverEnTrayectoria(puntoSobrePila(nivel), PAUSA_PASO_LIBRE_MS)) return false;

    return !detenerRutina;
}

/**
 * @brief Parte de la posicion de inicio, recorre las posiciones 1 a 3 y en cada una mide con el sensor.
 *        Si hay objeto, lo transporta y lo apila en la posicion final.
 *        Antes de cada ejecucion, la pila debe estar vacia.
 */
ResultadoRutina ejecutarRutinaApilado() {
    Enlace.println("\n[INFO] === INICIANDO RUTINA DE APILADO ===");

    if (!configuracionLista()) return RUTINA_CONFIG;
    if (!validarRutina()) {
        Enlace.println("[ERROR] Rutina cancelada: corrige las posiciones (o los limites) indicados arriba.");
        return RUTINA_ALCANCE;
    }

    estadoActual = "RUTINA";

    cubosApilados = 0;
    posicionActual = 0;
    bool error = false;

    // Pinza abierta y ejecutar pos_1 (movimiento articular desde la pose de inicio)
    moverServoSuavemente(PINZA, PINZA_ABIERTA, PAUSA_PASO_ARTICULAR_MS);
    if (!moverAPuntoArticular(puntoEscaneo(0))) error = true;

    for (int n = 0; n < CANTIDAD_POSICIONES && !error && !detenerRutina; n++) {
        // Medir con el sensor sobre la posicion actual
        posicionActual = n + 1;
        esperarMs(PAUSA_ESCANEO_MS);
        float distancia = medirDistanciaCm();
        ultimaDistanciaCm = distancia;

        Enlace.print("[Posicion ");
        Enlace.print(n + 1);
        Enlace.print("] Distancia: ");
        Enlace.print(distancia, 1);
        Enlace.print(" cm -> ");

        if (hayObjeto(distancia)) {
            Enlace.println("OBJETO DETECTADO");
            if (!transportarObjeto(n, cubosApilados)) {
                error = true;
                break;
            }
            cubosApilados++;
        } else {
            Enlace.println("vacia, se omite");
        }

        // pos_from_n_to_n+1 (tanto si se transporto un objeto como si no)
        if (n < CANTIDAD_POSICIONES - 1) {
            if (!moverEnTrayectoria(puntoEscaneo(n + 1), PAUSA_PASO_LIBRE_MS)) error = true;
        }
    }

    if (detenerRutina) {
        Enlace.println("[AVISO] Rutina detenida por el usuario. Usa HOME para volver a inicio.\n");
        return RUTINA_DETENIDA;
    }
    if (error) {
        Enlace.println("[ERROR] Rutina interrumpida. Revisa el mensaje anterior y usa HOME para volver a inicio.\n");
        return RUTINA_ERROR;
    }

    volverAInicio();
    if (detenerRutina) return RUTINA_DETENIDA;

    Enlace.print("[INFO] === RUTINA FINALIZADA: ");
    Enlace.print(cubosApilados);
    Enlace.println(" cubo(s) apilado(s) ===\n");
    return RUTINA_OK;
}