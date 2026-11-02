
// ==========================================
// POSES DE LA RUTINA  [CALIBRAR]
// ==========================================

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
