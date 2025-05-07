# 🚜 STM32-Carro-4WD: Sistema de Navegación Autónoma 🚜

<div align="center">
  <img src="https://raw.githubusercontent.com/felipegarcia130/STM32-Carro-4WD/main/assets/tractor.jpg" alt="Tractor Autónomo" width="600px">
  
  <p><em>Tractor autónomo con navegación inteligente para agricultura de precisión</em></p>
  
  [![GitHub stars](https://img.shields.io/github/stars/felipegarcia130/STM32-Carro-4WD?style=social)](https://github.com/felipegarcia130/STM32-Carro-4WD/stargazers)
  [![GitHub forks](https://img.shields.io/github/forks/felipegarcia130/STM32-Carro-4WD?style=social)](https://github.com/felipegarcia130/STM32-Carro-4WD/network/members)
  [![GitHub issues](https://img.shields.io/github/issues/felipegarcia130/STM32-Carro-4WD)](https://github.com/felipegarcia130/STM32-Carro-4WD/issues)
  [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
</div>

---

## 📝 Descripción del Proyecto

Sistema embebido avanzado de navegación autónoma aplicado a la agricultura de precisión, desarrollado con microcontrolador **STM32H755 dual-core**. Emplea múltiples protocolos de comunicación, sensores de alta precisión y algoritmos inteligentes para navegación por waypoints. Diseñado para revolucionar la eficiencia en operaciones agrícolas mediante automatización.

---

## ✨ Características Principales

<div align="center">
  <table>
    <tr>
      <td align="center"><b>🧭 Navegación por Waypoints</b></td>
      <td align="center"><b>🔄 Comunicación Multiprotocolo</b></td>
      <td align="center"><b>📍 Sensado de Posición</b></td>
    </tr>
    <tr>
      <td>Sistema de routing inteligente mediante coordenadas</td>
      <td>Integración de UART, SPI, I2C, CAN</td>
      <td>Implementación de NRF24L01 y sistema de encoder</td>
    </tr>
    <tr>
      <td align="center"><b>⚙️ Control de Movimiento</b></td>
      <td align="center"><b>🧠 Inteligencia Embebida</b></td>
      <td align="center"><b>🔋 Operación Autónoma</b></td>
    </tr>
    <tr>
      <td>PWM para control de servo y motor DC</td>
      <td>Algoritmos de navegación con toma de decisiones autónomas</td>
      <td>Batería LiPo para operación independiente</td>
    </tr>
  </table>
</div>

---

## 🗂️ Estructura del Repositorio

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

---

## 💻 Tecnologías Implementadas

<div align="center">
  <table>
    <tr>
      <th>Categoría</th>
      <th>Componentes</th>
      <th>Descripción</th>
    </tr>
    <tr>
      <td><b>Microcontrolador</b></td>
      <td>STM32H755</td>
      <td>Dual Core Cortex-M7/M4 con rendimiento excepcional</td>
    </tr>
    <tr>
      <td><b>Sensores</b></td>
      <td>MPU6050, Encoder</td>
      <td>IMU 6-DoF para orientación y encoder para medición de distancia</td>
    </tr>
    <tr>
      <td><b>Comunicación</b></td>
      <td>NRF24L01, TJA1051, MCP2515</td>
      <td>RF 2.4GHz, Transceptor CAN y Controlador CAN</td>
    </tr>
    <tr>
      <td><b>Actuadores</b></td>
      <td>Servomotor DS04-NFC, Motor DC GA25-370</td>
      <td>Control de dirección y tracción de alta precisión</td>
    </tr>
    <tr>
      <td><b>Control</b></td>
      <td>ESC-10A</td>
      <td>Electronic Speed Controller para control preciso del motor</td>
    </tr>
  </table>
</div>

### 🔄 Protocolos de Comunicación
- **I2C**: Comunicación con MPU6050
- **SPI**: Comunicación con NRF24L01
- **CAN**: Comunicación entre STM32 y Arduino (encoder)
- **UART**: Debugging y monitoreo de datos
- **PWM**: Control de servomotor y velocidad del motor DC

---

## 🚀 Funcionalidades Destacadas

### 1. 🧭 Navegación Autónoma
Implementación de algoritmos avanzados para seguimiento de rutas predefinidas mediante waypoints, utilizando coordenadas recibidas por el módulo NRF24L01 y respaldadas por el encoder.

```c
void NRFMotorEncoderCAN() {
    // Lógica de navegación que combina coordenadas del NRF y datos del encoder
    // para una navegación precisa entre waypoints
}
```

### 2. ⚙️ Control de Dirección Inteligente
Algoritmo de cálculo de ángulo basado en la posición actual y el punto objetivo, ajustando dinámicamente la velocidad según la complejidad de la curva.

```c
double targetAngle = atan2(targetsY[segmento] - coordY, targetsX[segmento] - coordX);
targetAngle = round(targetAngle * 180.0 / M_PI); // Convertimos radianes a grados
double angleError = targetAngle - (double)(angle);
```

### 3. 🛡️ Sistema de Redundancia
Implementación de múltiples sistemas de posicionamiento que garantizan la navegación precisa incluso cuando uno de los sistemas falla.

### 4. 📡 Comunicación Multimódulo
Integración de diversos protocolos para asegurar la comunicación eficiente entre todos los componentes del sistema.

---

## 🔄 Diagrama del Sistema

<div align="center">
  <pre>
  ┌─────────────┐         ┌──────────────┐         ┌─────────────┐
  │    STM32    │◄──I2C──►│   MPU6050    │         │             │
  │   (Master)  │◄──SPI──►│   NRF24L01   │◄──RF──► │    Cámara   │
  │             │◄──PWM──►│ Servo & Motor│         │  Tracking   │
  │             │◄──CAN──►│              │         │             │
  └─────────────┘         └──────────────┘         └─────────────┘
        ▲                                                 │
        │                                                 │
        ▼                                                 ▼
  ┌─────────────┐         ┌──────────────┐         ┌─────────────┐
  │   Arduino   │◄───────►│    Encoder   │         │    Punto    │
  │  (CAN Node) │         │              │         │   Objetivo  │
  └─────────────┘         └──────────────┘         └─────────────┘
  </pre>
</div>

---

## 🛠️ Instalación y Configuración

### Requisitos
- STM32CubeIDE v1.13.0 o superior
- Compilador GCC ARM
- Placa STM32H755 Discovery

### Pasos para Compilar
1. Clonar el repositorio: `git clone https://github.com/felipegarcia130/STM32-Carro-4WD.git`
2. Abrir el proyecto en STM32CubeIDE
3. Configurar la placa STM32H755
4. Compilar y flashear el firmware

---

## 📊 Resultados

<div align="center">
  <table>
    <tr>
      <th>Métrica</th>
      <th>Resultado</th>
    </tr>
    <tr>
      <td>Precisión de navegación</td>
      <td>±7cm en puntos objetivo</td>
    </tr>
    <tr>
      <td>Autonomía</td>
      <td>~4 horas con batería LiPo</td>
    </tr>
    <tr>
      <td>Tiempo respuesta</td>
      <td><100ms para ajustes de dirección</td>
    </tr>
    <tr>
      <td>Fiabilidad del sistema</td>
      <td>Navegación completa exitosa en múltiples pruebas</td>
    </tr>
  </table>
</div>

---

## 🎥 Demostración

<div align="center">
  <a href="https://drive.google.com/file/d/1VipUzo-c-BnRVilcmz3iSH7LOjSFaA0G/view" target="_blank">
    <img src="https://img.shields.io/badge/Ver%20Video-Demostración-red?style=for-the-badge&logo=youtube" alt="Ver Video">
  </a>
</div>

---

## 👨‍💻 Equipo de Desarrollo

<div align="center">
  <table>
    <tr>
      <th>Integrante</th>
      <th>Matrícula</th>
      <th>Contribución</th>
    </tr>
    <tr>
      <td><b>Felipe de Jesús García García</b></td>
      <td>A01705893</td>
      <td>Creación de carcasa y componentes 3D</td>
    </tr>
    <tr>
      <td><b>Alfonso Solís Díaz</b></td>
      <td>A00838034</td>
      <td>Implementación de módulos de comunicación, algoritmos de navegación avanzados y configuración del sistema</td>
    </tr>
    <tr>
      <td><b>Jesús René Hernández Galindo</b></td>
      <td>A00837617</td>
      <td>Construcción e implementación del hardware</td>
    </tr>
    <tr>
      <td><b>Juan José Castillo González</b></td>
      <td>A01750541</td>
      <td>Programación de rutas y algoritmos de navegación</td>
    </tr>
  </table>
</div>

---

## 📄 Licencia
Este proyecto está licenciado bajo [MIT License](LICENSE).

---

## 🙏 Agradecimientos
Un agradecimiento especial a **John Deere** por el patrocinio y al **Instituto Tecnológico y de Estudios Superiores de Monterrey** por proporcionar las instalaciones y equipamiento necesarios para desarrollar este proyecto.

---

<div align="center">
  <p>Desarrollado con ❤️ por el Equipo 5</p>
  <p>© 2025 - Todos los derechos reservados</p>
</div>
