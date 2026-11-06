/**
 * @file driver_wifi.cpp
 * @brief Implementacion del enlace Serial + WiFi (TCP).
 * @author Emilio Rojas
 */

#include "driver_wifi.h"

#include <ESPmDNS.h>

EnlaceApp Enlace;

// ==========================================
// ESTADO PRIVADO DEL MODULO
// ==========================================
namespace {

constexpr size_t TAMANO_BUFER_SALIDA = 256;

WiFiServer* servidor = nullptr;
WiFiClient cliente;

// Las lineas se acumulan y se envian completas (evita mandar un paquete por caracter)
uint8_t buferSalida[TAMANO_BUFER_SALIDA];
size_t largoBufer = 0;

bool modoPuntoDeAcceso = false;

}  // namespace

// ==========================================
// INICIALIZACION
// ==========================================

void EnlaceApp::iniciar(const char* ssid, const char* clave,
                        const char* apSsid, const char* apClave,
                        uint16_t puerto, unsigned long timeoutMs) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, clave);

    Serial.print("[WIFI] Conectando a ");
    Serial.print(ssid);

    const unsigned long inicio = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - inicio < timeoutMs) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        modoPuntoDeAcceso = false;
        Serial.print("[WIFI] Conectado. IP del robot: ");
        Serial.println(WiFi.localIP());
    } else {
        // Sin red: el robot crea la suya para poder usarlo igual
        modoPuntoDeAcceso = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAP(apSsid, apClave);
        Serial.print("[WIFI] No se pudo conectar. Punto de acceso '");
        Serial.print(apSsid);
        Serial.print("' creado. IP del robot: ");
        Serial.println(WiFi.softAPIP());
    }

    if (servidor == nullptr) servidor = new WiFiServer(puerto);
    servidor->begin();

    // Nombre en la red: brazo.local (no todos los telefonos lo resuelven; la IP siempre sirve)
    if (MDNS.begin("brazo")) {
        MDNS.addService("brazo", "tcp", puerto);
    }

    Serial.print("[WIFI] Servidor TCP en el puerto ");
    Serial.println(puerto);
}

// ==========================================
// CONEXIONES
// ==========================================

void EnlaceApp::atenderConexiones() {
    if (servidor == nullptr) return;

    WiFiClient nuevo = servidor->available();
    if (nuevo) {
        // Solo una app a la vez: la nueva reemplaza a la anterior
        if (cliente && cliente.connected()) cliente.stop();
        cliente = nuevo;
        cliente.setNoDelay(true);
        Serial.println("[WIFI] App conectada.");
        cliente.print("OK,CONECTADO\n");
    }

    if (cliente && !cliente.connected()) {
        cliente.stop();
        Serial.println("[WIFI] App desconectada.");
    }
}

bool EnlaceApp::appConectada() {
    return cliente && cliente.connected();
}

String EnlaceApp::direccionIp() {
    return modoPuntoDeAcceso ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
}

// ==========================================
// ENTRADA (Monitor Serie + WiFi)
// ==========================================

int EnlaceApp::available() {
    atenderConexiones();

    const int deSerie = Serial.available();
    if (deSerie > 0) return deSerie;
    if (appConectada()) return cliente.available();
    return 0;
}

int EnlaceApp::read() {
    if (Serial.available() > 0) return Serial.read();
    if (appConectada() && cliente.available() > 0) return cliente.read();
    return -1;
}

int EnlaceApp::peek() {
    if (Serial.available() > 0) return Serial.peek();
    if (appConectada() && cliente.available() > 0) return cliente.peek();
    return -1;
}

// ==========================================
// SALIDA (Monitor Serie + WiFi)
// ==========================================

void EnlaceApp::enviarPendiente() {
    if (largoBufer > 0 && appConectada()) {
        cliente.write(buferSalida, largoBufer);
    }
    largoBufer = 0;
}

size_t EnlaceApp::write(uint8_t dato) {
    Serial.write(dato);

    buferSalida[largoBufer++] = dato;
    if (dato == '\n' || largoBufer >= TAMANO_BUFER_SALIDA) enviarPendiente();
    return 1;
}

size_t EnlaceApp::write(const uint8_t* datos, size_t cantidad) {
    for (size_t i = 0; i < cantidad; i++) write(datos[i]);
    return cantidad;
}

void EnlaceApp::flush() {
    Serial.flush();
    enviarPendiente();
}
