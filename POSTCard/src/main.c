#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"   // Baldytech : libreria para regular el voltage
#include "lpc_bus_sniffer.pio.h"

// Baldytech : Asegura divisor de frecuencia de memoria a 2 para maximizar rendimiento a 132Mhz
#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

// Baldytech : incremento Mhz hasta 198 => 33MHz x 8, maximizamos potencia para procesar los frames LPC.
#define SYS_FREQ_IN_KHZ (198 * 1000)

// LAD[0-3] + LCLK + LFRAME, comenzando desde el pin GPIOX, donde X es el incial donde has colocado los cables.
#define LPC_BUS_PIN_BASE (5U)   // Comenzando desde el pin GPIO0
#define LPC_BUS_PIN_COUNT (6U)  // Un bloque de 6 pines en total

// --- CONFIGURACIÓN DEL DISPLAY TM1637 ---
#define TM_CLK_PIN 16
#define TM_DIO_PIN 17

// Tabla de codificación de 7 segmentos para caracteres Hexadecimales (0-F) más guion
const uint8_t segment_map[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, // 0, 1, 2, 3, 4, 5, 6, 7
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, // 8, 9, A, b, C, d, E, F
    0x40                                            // [16] Guion '-' para vacío
};

// --- DRIVER POR SOFTWARE (BIT-BANGING) PARA TM1637 ---
void tm1637_delay() {
    sleep_us(5); // Retardo estándar para la sincronización del bus TM1637
}

void tm1637_start() {
    gpio_set_dir(TM_DIO_PIN, GPIO_OUT);
    gpio_put(TM_DIO_PIN, 1);
    gpio_put(TM_CLK_PIN, 1);
    tm1637_delay();
    gpio_put(TM_DIO_PIN, 0);
    tm1637_delay();
    gpio_put(TM_CLK_PIN, 0);
    tm1637_delay();                      // añadido para no saturar TM1637
}

void tm1637_stop() {
    gpio_set_dir(TM_DIO_PIN, GPIO_OUT);
    gpio_put(TM_DIO_PIN, 0);
    gpio_put(TM_CLK_PIN, 1);
    tm1637_delay();
    gpio_put(TM_DIO_PIN, 1);
    tm1637_delay();
}

bool tm1637_write_byte(uint8_t byte) {
    gpio_set_dir(TM_DIO_PIN, GPIO_OUT);
    for (int i = 0; i < 8; i++) {
        gpio_put(TM_CLK_PIN, 0);
        tm1637_delay();                   // añadido para no saturar TM1637
        gpio_put(TM_DIO_PIN, byte & 0x01);
        byte >>= 1;
        tm1637_delay();
        gpio_put(TM_CLK_PIN, 1);
        tm1637_delay();
    }
    // Procesar bit de ACK del TM1637
    gpio_put(TM_CLK_PIN, 0);
    gpio_set_dir(TM_DIO_PIN, GPIO_IN);
    tm1637_delay();
    gpio_put(TM_CLK_PIN, 1);
    tm1637_delay();
    bool ack = (gpio_get(TM_DIO_PIN) == 0);
    gpio_put(TM_CLK_PIN, 0);
    tm1637_delay();                      // añadido para no saturar TM1637
    return ack;
}

void tm1637_init_display() {
    gpio_init(TM_CLK_PIN);
    gpio_init(TM_DIO_PIN);
    gpio_set_dir(TM_CLK_PIN, GPIO_OUT);
    gpio_set_dir(TM_DIO_PIN, GPIO_OUT);
    gpio_put(TM_CLK_PIN, 1);     // adición de la IA
    gpio_put(TM_DIO_PIN, 1);     // adición de la IA
    gpio_disable_pulls(TM_CLK_PIN);
    gpio_disable_pulls(TM_DIO_PIN);

    // Encender la pantalla mostrando inicialmente cuatro guiones "----"
    tm1637_start();
    tm1637_write_byte(0x40); // Dirección de datos automática
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(0xC0); // Comenzar desde el primer dígito (Izquierda)
    for (int i = 0; i < 4; i++) {
        tm1637_write_byte(segment_map[16]); // Enviar byte del guion
    }
    tm1637_stop();

    tm1637_start();
    tm1637_write_byte(0x8F); // Control del display: Encendido + Brillo Máximo
    tm1637_stop();
}

// Envía un byte en formato Hexadecimal en los dos dígitos de la derecha del display
void tm1637_display_post_code(uint8_t code) {
    uint8_t nibble_alto = (code >> 4) & 0x0F;
    uint8_t nibble_bajo = code & 0x0F;

   
    tm1637_start();
    tm1637_write_byte(0x44); // Modo de dirección fija
    tm1637_stop();

// Actualizar dígito 2 (Decenas en Hex)
    tm1637_start();
    tm1637_write_byte(0xC2); 
    tm1637_write_byte(segment_map[nibble_alto]);
    tm1637_stop();

// Actualizar dígito 3 (Unidades en Hex)
    tm1637_start();
    tm1637_write_byte(0xC3); 
    tm1637_write_byte(segment_map[nibble_bajo]);
    tm1637_stop();

}

// ----------------------------------------------------

typedef union __attribute__((__packed__)) {
  struct {
    uint16_t addr : 16;
    uint8_t cy_dir : 4;
    uint8_t start : 4;
    uint16_t resv : 8;
  }val;
  uint32_t value;
}lpcIOFrame;

void gpio_initialization() {
    for(uint i = LPC_BUS_PIN_BASE; i < LPC_BUS_PIN_BASE + LPC_BUS_PIN_COUNT; i++) {
        // Inicializa los pines LAD[0-3] + LCLK + LFRAME.
        gpio_init(i);
        // Desactiva las resistencias internas de pull-up y pull-down.
        gpio_disable_pulls(i);
    }
}

uint32_t reverse_nibbles(uint32_t in_val) {
    uint32_t out_val = 0;

    for(uint8_t i = 0; i < 32; i += 4) {
        out_val <<= 4;
        out_val |= in_val >> i & 0xf;
    }
    return out_val;
}

int init_lpc_bus_sniffer(PIO pio) {
    uint offset;

    int sm = pio_claim_unused_sm(pio, true);
    if(sm < 0){
        return -1;
    }

    if (pio_can_add_program(pio, &lpc_bus_sniffer_program)) {
        offset = pio_add_program(pio, &lpc_bus_sniffer_program);
    } else {
        return -2;
    }

    lpc_bus_sniffer_program_init(pio, sm, offset, LPC_BUS_PIN_BASE, LPC_BUS_PIN_COUNT, 0, 0);
    pio_sm_set_enabled(pio, sm, true);

    lpcIOFrame filter; 
    filter.value = 0;
    filter.val.start = 0x0;
    filter.val.cy_dir = 0x2;
    filter.val.addr = 0x80; // Escuchar puerto POST de la placa base
    
    pio->txf[sm] = reverse_nibbles(filter.value);

    return sm;
}

int main() {
    // Baldytech : sube el voltage a 1,2v (de 1,1)
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);

    set_sys_clock_khz(SYS_FREQ_IN_KHZ, true);    // Configura la frecuencia del sistema.
    gpio_initialization();                       // Inicializa los pines GPIO del bus.
    tm1637_init_display();                       // Inicializa la pantalla TM1637


    // Inicializa el bloque PIO
    PIO _pio0 = pio0;
    int sm_lpc_bus_sniffer = init_lpc_bus_sniffer(_pio0);
    if (sm_lpc_bus_sniffer < 0) return -1;

    uint32_t ultimo_codigo = 0xFFFFFFFF; // Guarda el valor previo para detectar cambios

    while(1) {
        if (!pio_sm_is_rx_fifo_empty(_pio0, sm_lpc_bus_sniffer)) {
            uint32_t rxdata = _pio0->rxf[sm_lpc_bus_sniffer];
            
            // Si el código capturado es diferente al anterior, actualiza la pantalla
            if (rxdata != ultimo_codigo) {
                ultimo_codigo = rxdata;
                
                // Muestra el código en el display TM1637 (Solo los 8 bits de datos del puerto 0x80)
                tm1637_display_post_code((uint8_t)rxdata);
            }
        }
    }
}