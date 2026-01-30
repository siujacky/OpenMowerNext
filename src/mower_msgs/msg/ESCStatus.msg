# ESC (Electronic Speed Controller) status message
# Reports status of motor controllers

# Status constants
uint8 ESC_STATUS_DISCONNECTED = 99
uint8 ESC_STATUS_ERROR = 100
uint8 ESC_STATUS_STALLED = 150
uint8 ESC_STATUS_OK = 200
uint8 ESC_STATUS_RUNNING = 201

uint8 status                 # Current ESC status (one of the constants above)
float32 current              # Motor current draw [A]
uint32 tacho                 # Tachometer count
int16 rpm                    # Motor RPM
float32 temperature_motor    # Motor temperature [°C]
float32 temperature_pcb      # PCB temperature [°C]
