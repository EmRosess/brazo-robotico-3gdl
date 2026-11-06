
// ##########################################################################
// >>> ARCHIVO 9: PickAndPlace.ino
// ##########################################################################
#include "config.h"
#include "Driver_servos.h"
#include "Driver_sensor.h"
#include "Modelo_cinematica.h"
#include "Controlador_comandos.h"
#include "Modelo_brazo.h"
#include "Vista_telemetria.h"
#include "driver_wifi.h"


void setup() {
    vistaIniciar();
    iniciarSensor();
    iniciarServos();
    servicioDeFondo = servicioDuranteMovimiento;   // Telemetria y STOP durante los movimientos
    Enlace.iniciar(WIFI_SSID, WIFI_CLAVE, AP_SSID, AP_CLAVE, WIFI_PUERTO, WIFI_TIMEOUT_MS);
    autopruebaCinematica();

    if (configuracionLista()) {
        Serial.println("[OK] Configuracion completa.");
    }
    mostrarAyuda();
    emitirTelemetria();
}

void loop() {
    emitirTelemetriaPeriodica();
    procesarPuertoSerie();
}
