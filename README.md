# STM32-Carro-4WD: Sistema de Navegación Autónoma

![Tractor Autónomo](https://raw.githubusercontent.com/felipegarcia130/STM32-Carro-4WD/main/assets/tractor.jpg)

## Descripción del Proyecto
Sistema embebido avanzado para navegación autónoma aplicado a la agricultura de precisión, basado en un tractor 4WD implementado con microcontrolador STM32H755. El proyecto integra múltiples protocolos de comunicación, sensores y actuadores para lograr una navegación precisa por waypoints predefinidos.

## Características Principales

🔸 **Navegación por Waypoints**: Sistema de routing inteligente mediante coordenadas
🔸 **Comunicación Multiprotocolo**: Integración de UART, SPI, I2C, CAN
🔸 **Sensado de Posición**: Implementación de NRF24L01 y sistema de encoder
🔸 **Control de Movimiento Preciso**: PWM para control de servo y motor DC
🔸 **Inteligencia Embebida**: Algoritmos de navegación con toma de decisiones autónomas

## Estructura del Repositorio

```
STM32-Carro-4WD/
├── Core/                       # Núcleo del proyecto
│   ├── Inc/                    # Archivos de cabecera
│   │   ├── main.h              # Definiciones principales
│   │   ├── nrf24.h             # Driver de NRF24L01
│   │   ├── mpu6050.h           # Driver de MPU6050
│   │   └── ...
│   ├── Src/                    # Código fuente
│       ├── main.c              # Lógica principal
│       ├── nrf24.c             # Implementación de comunicación inalámbrica
│       ├── mpu6050.c           # Implementación de IMU
│       └── ...
├── Drivers/                    # Drivers STM32
├── assets/                     # Imágenes y recursos
├── docs/                       # Documentación adicional
└── README.md                   # Este archivo
```

## Tecnologías Implementadas

### Hardware
- **Microcontrolador**: STM32H755 (Dual Core Cortex-M7/M4)
- **Sensores**: MPU6050 (IMU)
- **Comunicación**: NRF24L01, TJA1051 (CAN Transceiver), MCP2515 (CAN Controller)
- **Actuadores**: Servomotor DS04-NFC, Motor DC GA25-370 con encoder
- **Control**: ESC-10A (Electronic Speed Controller)
- **Energía**: Batería LiPo 7.4V

### Protocolos de Comunicación
- **I2C**: Comunicación con MPU6050
- **SPI**: Comunicación con NRF24L01
- **CAN**: Comunicación entre STM32 y Arduino (encoder)
- **UART**: Debugging y monitoreo de datos
- **PWM**: Control de servomotor y velocidad del motor DC

## Funcionalidades Destacadas

### 1. Navegación Autónoma
Implementación de algoritmos avanzados para seguimiento de rutas predefinidas mediante waypoints, utilizando coordenadas recibidas por el módulo NRF24L01 y respaldadas por el encoder.

```c
void NRFMotorEncoderCAN() {
    // Lógica de navegación que combina coordenadas del NRF y datos del encoder
    // para una navegación precisa entre waypoints
}
```

### 2. Control de Dirección Inteligente
Algoritmo de cálculo de ángulo basado en la posición actual y el punto objetivo, ajustando dinámicamente la velocidad según la complejidad de la curva.

```c
double targetAngle = atan2(targetsY[segmento] - coordY, targetsX[segmento] - coordX);
targetAngle = round(targetAngle * 180.0 / M_PI); // Convertimos radianes a grados
double angleError = targetAngle - (double)(angle);
```

### 3. Sistema de Redundancia
Implementación de múltiples sistemas de posicionamiento que garantizan la navegación precisa incluso cuando uno de los sistemas falla.

### 4. Comunicación Multimódulo
Integración de diversos protocolos para asegurar la comunicación eficiente entre todos los componentes del sistema.

## Diagrama del Sistema

```
+-------------+         +--------------+         +-------------+
|    STM32    |<--I2C-->|   MPU6050    |         |             |
|   (Master)  |<--SPI-->|   NRF24L01   |<-RF->   |    Cámara   |
|             |<--PWM-->| Servo & Motor|         |  Tracking   |
|             |<--CAN-->|              |         |             |
+-------------+         +--------------+         +-------------+
      ^                                                 |
      |                                                 |
      v                                                 v
+-------------+         +--------------+         +-------------+
|   Arduino   |<------->|    Encoder   |         |    Punto    |
|  (CAN Node) |         |              |         |   Objetivo  |
+-------------+         +--------------+         +-------------+
```

## Instalación y Configuración

### Requisitos
- STM32CubeIDE v1.13.0 o superior
- Compilador GCC ARM
- Placa STM32H755 Discovery

### Pasos para Compilar
1. Clonar el repositorio: `git clone https://github.com/felipegarcia130/STM32-Carro-4WD.git`
2. Abrir el proyecto en STM32CubeIDE
3. Configurar la placa STM32H755
4. Compilar y flashear el firmware

## Resultados
El sistema logra navegar de forma autónoma entre waypoints con una precisión de ±7cm, ajustando su velocidad y dirección automáticamente según las condiciones del recorrido.

## Demostración

[![Video Demostración](https://img.youtube.com/vi/YOUTUBE_VIDEO_ID/0.jpg)](https://drive.google.com/file/d/1VipUzo-c-BnRVilcmz3iSH7LOjSFaA0G/view)

## Equipo de Desarrollo

| Integrante | Matrícula | Contribución |
|------------|-----------|--------------|
| Felipe de Jesús García García | A01705893 | Creación de carcasa y componentes 3D |
| Alfonso Solís Díaz | A00838034 | Implementación de módulos y configuración |
| Jesús René Hernández Galindo | A00837617 | Construcción e implementación del hardware |
| Juan José Castillo González | A01750541 | Programación de rutas y algoritmos de navegación |

## Licencia
Este proyecto está licenciado bajo [MIT License](LICENSE).

## Agradecimientos
Un agradecimiento especial a John Deere por el patrocinio y al Instituto Tecnológico y de Estudios Superiores de Monterrey por proporcionar las instalaciones y equipamiento necesarios para desarrollar este proyecto.
