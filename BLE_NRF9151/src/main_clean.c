#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nrf91, LOG_LEVEL_INF);
#include <string.h>  // Necesario para strcmp
#include <strings.h> // Necesario para strcasecmp

// BME680 Sensor - usando driver estándar de Zephyr
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/__assert.h>
// #include <drivers/bme68x_iaq.h>  // NO disponible - falta módulo BME68X

// * MQTT y certificados

#include <stdio.h>
#include <ncs_version.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>

#include <dk_buttons_and_leds.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>

#include "mqtt_connection.h"

#if NCS_VERSION_NUMBER < 0x20600
#include <zephyr/random/rand32.h>
#else 
#include <zephyr/random/random.h>
#endif

// * GNSS y CoAP
// #include "gps_coap.h"

void mqtt_publish_bmi270(float ax, float ay, float az, float gx, float gy, float gz);

/* The mqtt client struct */
static struct mqtt_client client;
/* File descriptor */
static struct pollfd fds;

static K_SEM_DEFINE(lte_connected, 0, 1);

int connected = 0; // Variable para indicar conexión al MQTT

static void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type)
    {
    case LTE_LC_EVT_NW_REG_STATUS:
        if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
            (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING))
        {
            break;
        }
        LOG_INF("Network registration status: %s",
                evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "Connected - home network" : "Connected - roaming");
        k_sem_give(&lte_connected);
        break;
    case LTE_LC_EVT_RRC_UPDATE:
        LOG_INF("RRC mode: %s", evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
        break;
    default:
        break;
    }
}

static int modem_configure(void)
{
    int err;

    LOG_INF("Initializing modem library");

    err = nrf_modem_lib_init();
    if (err)
    {
        LOG_ERR("Failed to initialize the modem library, error: %d", err);
        return err;
    }

    /* STEP 4.3 - Store the certificate in the modem while the modem is in offline mode  */
    err = certificate_provision();
    if (err)
    {
        LOG_ERR("Failed to provision certificates");
        return err;
    }

    LOG_INF("Connecting to LTE network");
    err = lte_lc_connect_async(lte_handler);
    if (err)
    {
        LOG_ERR("Error in lte_lc_connect_async, error: %d", err);
        return err;
    }

    k_sem_take(&lte_connected, K_FOREVER);
    LOG_INF("Connected to LTE network");
    dk_set_led_on(DK_LED2);

    return 0;
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    switch (has_changed)
    {
    case DK_BTN1_MSK:
        if (button_state & DK_BTN1_MSK)
        {
            int err = data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE,
                                   CONFIG_BUTTON_EVENT_PUBLISH_MSG, sizeof(CONFIG_BUTTON_EVENT_PUBLISH_MSG) - 1,
                                   CONFIG_MQTT_PUB_TOPIC);
            if (err)
            {
                LOG_INF("Failed to send message, %d", err);
                return;
            }
        }
        break;
    }
}

int  mqtt_loop(void)
{
    int err = poll(&fds, 1, mqtt_keepalive_time_left(&client));
    if (err < 0)
    {
        LOG_ERR("Error in poll(): %d", errno);
        return 1;
    }

    err = mqtt_live(&client);
    if ((err != 0) && (err != -EAGAIN))
    {
        LOG_ERR("Error in mqtt_live: %d", err);
        return 1;
    }

    if ((fds.revents & POLLIN) == POLLIN)
    {
        err = mqtt_input(&client);
        if (err != 0)
        {
            LOG_ERR("Error in mqtt_input: %d", err);
            return 1;
        }
    }

    if ((fds.revents & POLLERR) == POLLERR)
    {
        LOG_ERR("POLLERR");
        return 1;
    }

    if ((fds.revents & POLLNVAL) == POLLNVAL)
    {
        LOG_ERR("POLLNVAL");
        return 1;
    }

    return 0;
}

// * MQTT y certificados

struct sensor_value acc[3], gyr[3];
struct sensor_value full_scale, sampling_freq, oversampling;
const struct device *const bmi270_dev = DEVICE_DT_GET_ONE(bosch_bmi270); // BMI270 device
int bmi270_is_working = 0;                                               // Corregido: BMI270 (acelerómetro/giroscopio), no BME270

// LED del nRF9151 - Solo usar led0 (P0.29 según overlay)
#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

// UART para comunicación con nRF5340
#define UART_NODE DT_NODELABEL(uart0)
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

// BME68X sensor device
static const struct device *bme680_dev = DEVICE_DT_GET_ANY(bosch_bme680); // BME680 device

// Timer y work para BME680 cada 2 segundos
static struct k_timer bme680_timer;
static struct k_work bme680_realtime_work;

// Estado del LED
static bool led_state = false;

#define UART_RX_BUF_SIZE 32 // Suficiente para "LED_OFF\n\0" y margen adicional
static char uart_rx_buf[UART_RX_BUF_SIZE];
static int uart_rx_buf_pos = 0;

// Buffer para detectar comandos binarios BMI270 del nRF53
#define BMI270_BINARY_BUF_SIZE 16
static uint8_t bmi270_binary_buf[BMI270_BINARY_BUF_SIZE];
static int bmi270_binary_pos = 0;

// Work queue para procesar comandos de sensores fuera del contexto de interrupción
static struct k_work bme_work;
static struct k_work bmi_work;

// Timer y work para BMI270 en modo tiempo real (más eficiente)
static struct k_timer bmi270_timer;
static struct k_work bmi270_realtime_work;

// Timer y work para BME680 cada 2 segundos
static struct k_timer bme680_timer;
static struct k_work bme680_realtime_work;

// Forward declarations - Declaraciones de funciones
static void control_led(bool turn_on);
static void get_data_bme680(const struct device *dev);
void get_data_bmi270(void);
void start_bme680(const struct device *dev);
int start_bmi270(void); // Corregido: BMI270, no BME270
static void bme_work_handler(struct k_work *work);
static void bmi_work_handler(struct k_work *work);
// Nuevas funciones para manejo eficiente del BMI270
static void bmi270_timer_handler(struct k_timer *timer);
static void bmi270_realtime_work_handler(struct k_work *work);
static void start_bmi270_realtime(void);
static void stop_bmi270_realtime(void);
// Función para detectar comandos binarios BMI270 del nRF53
static bool detect_bmi270_binary_command(uint8_t byte);

// Timer handlers para el modo automático
static void bmi270_timer_handler(struct k_timer *timer);
static void bme680_timer_handler(struct k_timer *timer);

/**
 * @brief Controlar LED basado en comando binario
 */
static void control_led(bool turn_on)
{
    LOG_INF("-> nrf91: 🎯 LED Command: %s", turn_on ? "ON" : "OFF");

    if (!gpio_is_ready_dt(&led))
    {
        LOG_ERR("-> nrf91: ❌ LED not ready");
        return;
    }

    int val_to_set = turn_on ? 1 : 0;
    int ret = gpio_pin_set_dt(&led, val_to_set);

    if (ret < 0)
    {
        LOG_ERR("-> nrf91: Failed to set LED state, error: %d", ret);
    }
    else
    {
        LOG_INF("-> nrf91: gpio_pin_set_dt(P0.29, %d) called for %s",
                val_to_set, turn_on ? "ON" : "OFF");
    }

    led_state = turn_on; // Actualizar estado lógico
    LOG_INF("-> nrf91: ✅ LED %s command processed", turn_on ? "ON" : "OFF");
}

// Work function para BME680 cada 2 segundos
static void bme680_realtime_work_handler(struct k_work *work)
{
    if (device_is_ready(bme680_dev))
    {
        if (sensor_sample_fetch(bme680_dev) == 0)
        {
            struct sensor_value temp, hum, pres, gas;

            sensor_channel_get(bme680_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
            sensor_channel_get(bme680_dev, SENSOR_CHAN_HUMIDITY, &hum);
            sensor_channel_get(bme680_dev, SENSOR_CHAN_PRESS, &pres);
            sensor_channel_get(bme680_dev, SENSOR_CHAN_GAS_RES, &gas);

            double temperature = sensor_value_to_double(&temp);
            double humidity = sensor_value_to_double(&hum);
            double pressure = sensor_value_to_double(&pres);
            double gas_resistance = sensor_value_to_double(&gas);

            LOG_INF("BME680 - Temp: %.2f°C Humidity: %.1f%% Pressure: %.1f kPa Gas: %.1f Ohms",
                    temperature, humidity, pressure, gas_resistance);

            // Publicar datos del BME680 por MQTT
            mqtt_publish_bme680(temperature, humidity, pressure, gas_resistance);
        }
    }
}
static bool detect_bmi270_binary_command(uint8_t byte)
{
    // Agregar byte al buffer binario
    if (bmi270_binary_pos < BMI270_BINARY_BUF_SIZE - 1)
    {
        bmi270_binary_buf[bmi270_binary_pos++] = byte;
    }
    else
    {
        // Buffer lleno - hacer shift y agregar nuevo byte
        for (int i = 0; i < BMI270_BINARY_BUF_SIZE - 1; i++)
        {
            bmi270_binary_buf[i] = bmi270_binary_buf[i + 1];
        }
        bmi270_binary_buf[BMI270_BINARY_BUF_SIZE - 1] = byte;
    }

    // Patrón 1: "XJ" seguido de bytes específicos (detectado en el log)
    if (bmi270_binary_pos >= 2)
    {
        if (bmi270_binary_buf[0] == 0x58 && bmi270_binary_buf[1] == 0x4A)
        { // 'X' 'J'
            LOG_INF("-> nrf91: 🎯 BMI270 BINARY PATTERN DETECTED: XJ sequence!");
            LOG_INF("-> nrf91: ✅ Activating BMI270 sensor from binary command");
            bmi270_binary_pos = 0; // Reset buffer
            return true;
        }
    }

    // Patrón 2: Secuencia específica que incluye 0xFF (detectada en el log)
    if (bmi270_binary_pos >= 3)
    {
        bool has_x = false, has_j = false, has_ff = false;
        for (int i = 0; i < bmi270_binary_pos; i++)
        {
            if (bmi270_binary_buf[i] == 0x58)
                has_x = true;
            if (bmi270_binary_buf[i] == 0x4A)
                has_j = true;
            if (bmi270_binary_buf[i] == 0xFF)
                has_ff = true;
        }

        if (has_x && has_j && has_ff)
        {
            LOG_INF("-> nrf91: 🎯 BMI270 BINARY PATTERN DETECTED: XJ+FF sequence!");
            LOG_INF("-> nrf91: ✅ Activating BMI270 sensor from extended binary command");
            bmi270_binary_pos = 0; // Reset buffer
            return true;
        }
    }

    // Reset buffer si está lleno y no se detectó patrón
    if (bmi270_binary_pos >= BMI270_BINARY_BUF_SIZE)
    {
        bmi270_binary_pos = 0;
    }

    return false;
}

/**
 * @brief Work handler para leer BME680 fuera del contexto de interrupción
 */
static void bme_work_handler(struct k_work *work)
{
    LOG_INF("-> nrf91: 🌡️ Processing BME680 data request");
    get_data_bme680(bme680_dev);
}

/**
 * @brief Work handler para leer BMI270 fuera del contexto de interrupción
 * ACTUALIZADO para usar el método EFICIENTE con timer
 */
static void bmi_work_handler(struct k_work *work)
{
    LOG_INF("-> nrf91: 🏃 Processing BMI270 data request - EFFICIENT mode");
    start_bmi270_realtime(); // Usar método eficiente con timer
}

/**
 * @brief Callback UART - Procesar comandos BINARIOS y de TEXTO del nRF5340
 * ACTUALIZADO: Detecta comandos BMI270 binarios del nRF53
 */
static void uart_callback(const struct device *dev, void *user_data)
{
    uint8_t byte;

    if (!uart_irq_update(uart_dev))
    {
        return;
    }

    LOG_INF("-> nrf91:  --> 💬 uart_callback  ");

    while (uart_irq_rx_ready(uart_dev))
    {
        uart_fifo_read(uart_dev, &byte, 1);

        LOG_INF("-> nrf91: 📥 RAW BYTE: 0x%02X ('%c')", byte, (byte >= 32 && byte <= 126) ? byte : '?');

        // MODO 1: Comandos binarios (para compatibilidad con nRF53 actual)
        if (byte == 0x01)
        {
            LOG_INF("-> nrf91: ✅ BINARY Command 0x01 (ON) - Turning LED ON");
            control_led(true);
            uart_rx_buf_pos = 0; // Reset buffer
            continue;
        }
        else if (byte == 0x00)
        {
            // 0x00 podría ser comando OFF o corrupción de datos
            // Solo lo procesamos si el buffer está vacío (comando directo)
            if (uart_rx_buf_pos == 0)
            {
                LOG_INF("-> nrf91: ✅ BINARY Command 0x00 (OFF) - Turning LED OFF");
                control_led(false);
                continue;
            }
            // Si hay datos en el buffer, ignoramos este 0x00 como posible corrupción
            LOG_WRN("-> nrf91: ⚠️ Ignoring 0x00 (possible corruption, buffer has %d chars)", uart_rx_buf_pos);
            continue;
        }
        // DETECCIÓN DE COMANDOS BMI270 BINARIOS del nRF53
        // Usar función inteligente para detectar patrones binarios
        if (detect_bmi270_binary_command(byte))
        {
            LOG_INF("-> nrf91: � BMI270 binary command confirmed - Starting real-time mode");
            k_work_submit(&bmi_work);
            uart_rx_buf_pos = 0; // Reset text buffer también
            continue;
        }

        // MODO 2: Comandos de texto - acumular caracteres
        // Si es un carácter de fin de línea, procesar el búfer
        if (byte == '\n' || byte == '\r' || uart_rx_buf_pos >= (UART_RX_BUF_SIZE - 1))
        {
            if (uart_rx_buf_pos > 0)
            {                                        // Solo procesar si hay algo en el búfer
                uart_rx_buf[uart_rx_buf_pos] = '\0'; // Null-terminar la cadena
                LOG_INF("-> nrf91: 💬 TEXT Command: \"%s\"", uart_rx_buf);

                // Comandos con formato LED_ON / LED_OFF
                if (strcmp(uart_rx_buf, "LED_ON") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"LED_ON\" - Turning LED ON");
                    control_led(true);
                }
                else if (strcmp(uart_rx_buf, "LED_OFF") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"LED_OFF\" - Turning LED OFF");
                    control_led(false);
                }
                // Comandos con formato ON / OFF (compatibilidad)
                else if (strcmp(uart_rx_buf, "ON") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"ON\" - Turning LED ON");
                    control_led(true);
                }
                else if (strcmp(uart_rx_buf, "OFF") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"OFF\" - Turning LED OFF");
                    control_led(false);
                }
                // Comando para solicitar datos del BME680
                else if (strcmp(uart_rx_buf, "BME_DATA") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"BME_DATA\" - Scheduling BME680 read");
                    k_work_submit(&bme_work);
                }
                // Comando para solicitar datos del BMI270
                else if (strcmp(uart_rx_buf, "BMI_DATA") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"BMI_DATA\" - Scheduling BMI270 read");
                    k_work_submit(&bmi_work);
                }
                // Comando para detener el modo tiempo real del BMI270
                else if (strcmp(uart_rx_buf, "BMI_STOP") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"BMI_STOP\" - Stopping BMI270 real-time mode");
                    stop_bmi270_realtime(); // Usar método eficiente
                }
                // Detectar comando "XJ" como texto también (respaldo)
                else if (strcmp(uart_rx_buf, "XJ") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"XJ\" - BMI270 Binary pattern as text - Activating sensor");
                    k_work_submit(&bmi_work);
                }
                // Comando "bmi270" (case-insensitive) para activar BMI270
                else if (strcasecmp(uart_rx_buf, "bmi270") == 0)
                {
                    LOG_INF("-> nrf91: ✅ TEXT Command \"bmi270\" - Activating BMI270 real-time mode");
                    k_work_submit(&bmi_work);
                }
                else
                {
                    LOG_WRN("-> nrf91: ❓ Unknown TEXT Command: \"%s\"", uart_rx_buf);
                }
            }
            uart_rx_buf_pos = 0; // Reset para el próximo comando
        }
        else if (byte >= 32 && byte <= 126)
        { // Solo caracteres imprimibles
            // Añadir el byte al búfer si es un carácter imprimible
            if (uart_rx_buf_pos < (UART_RX_BUF_SIZE - 1))
            {
                uart_rx_buf[uart_rx_buf_pos++] = (char)byte;
            }
        }
        else
        {
            // Carácter no imprimible - podría ser parte de comando binario BMI270
            // La función detect_bmi270_binary_command ya fue llamada arriba
            // Solo logear ocasionalmente para no saturar los logs
            static int non_printable_count = 0;
            non_printable_count++;
            if (non_printable_count % 5 == 0)
            {
                LOG_INF("-> nrf91: 🔍 Non-printable bytes received (count: %d) - checking for BMI270 patterns",
                        non_printable_count);
            }
        }
    }
}

// const struct sensor_trigger trig = {
//     .chan = SENSOR_CHAN_ALL,
//     .type = SENSOR_TRIG_TIMER,
// };

// Trigger handler para el sensor BME680 (deshabilitado por ahora)
// #if defined(CONFIG_APP_TRIGGER)
// static void trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
// {
//     struct sensor_value temp, press, humidity;
//
//     sensor_sample_fetch(dev);
//     sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
//     sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
//     sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
//
//     LOG_INF("sensor bme680 -> temp: %d.%06d°C; press: %d.%06d Pa; humidity: %d.%06d%%",
//             temp.val1, temp.val2, press.val1, press.val2, humidity.val1, humidity.val2);
// }
// #endif

static void get_data_bme680(const struct device *dev)
{
    struct sensor_value temp, press, humidity;
    // IAQ, CO2, VOC no disponibles en driver BME680 estándar
    // struct sensor_value iaq, co2, voc;

    sensor_sample_fetch(dev);
    sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
    sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
    // sensor_channel_get(dev, SENSOR_CHAN_IAQ, &iaq);
    // sensor_channel_get(dev, SENSOR_CHAN_CO2, &co2);
    // sensor_channel_get(dev, SENSOR_CHAN_VOC, &voc);

    LOG_INF("-> BME680: temp: %d.%06d°C; press: %d.%06d Pa; humidity: %d.%06d%%",
            temp.val1, temp.val2, press.val1, press.val2, humidity.val1, humidity.val2);
    // LOG_INF("-> BME680: iaq: %d; CO2: %d.%06d; VOC: %d.%06d",
    //         iaq.val1, co2.val1, co2.val2, voc.val1, voc.val2);
}

void start_bme680(const struct device *dev)
{

    LOG_INF("App started");

    k_sleep(K_SECONDS(5));

    if (dev == NULL)
    {
        LOG_ERR("--? no device found bosch_bme680");
        return;
    }
    if (!device_is_ready(dev))
    {
        LOG_ERR("--? device is not ready bosch_bme680");
        return;
    }

    LOG_ERR("--> funcionando bme680");
}

int start_bmi270() // Corregido: BMI270, no BME270
{
    // bmi270_dev

    if (!device_is_ready(bmi270_dev))
    {
        printf("Device %s is not ready\n", bmi270_dev->name);
        return 0;
    }

    printf("Device %p name is %s\n", bmi270_dev, bmi270_dev->name);

    /* Setting scale in G, due to loss of precision if the SI unit m/s^2
     * is used
     */
    full_scale.val1 = 2; /* G */
    full_scale.val2 = 0;
    sampling_freq.val1 = 100; /* Hz. Performance mode */
    sampling_freq.val2 = 0;
    oversampling.val1 = 1; /* Normal mode */
    oversampling.val2 = 0;

    sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
                    &full_scale);
    sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,
                    &oversampling);
    /* Set sampling frequency last as this also sets the appropriate
     * power mode. If already sampling, change to 0.0Hz before changing
     * other attributes
     */
    sensor_attr_set(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
                    SENSOR_ATTR_SAMPLING_FREQUENCY,
                    &sampling_freq);

    /* Setting scale in degrees/s to match the sensor scale */
    full_scale.val1 = 500; /* dps */
    full_scale.val2 = 0;
    sampling_freq.val1 = 100; /* Hz. Performance mode */
    sampling_freq.val2 = 0;
    oversampling.val1 = 1; /* Normal mode */
    oversampling.val2 = 0;

    sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE,
                    &full_scale);
    sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING,
                    &oversampling);
    /* Set sampling frequency last as this also sets the appropriate
     * power mode. If already sampling, change sampling frequency to
     * 0.0Hz before changing other attributes
     */
    sensor_attr_set(bmi270_dev, SENSOR_CHAN_GYRO_XYZ,
                    SENSOR_ATTR_SAMPLING_FREQUENCY,
                    &sampling_freq);
    return 1;
}

void get_data_bmi270()
{
    // Solo leer y mostrar datos si estamos en modo tiempo real
    if (bmi270_is_working != 2)
    {
        return; // Salir silenciosamente si no está en modo tiempo real
    }

    sensor_sample_fetch(bmi270_dev);
    sensor_channel_get(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    sensor_channel_get(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, gyr);

    // Convertir valores sensor_value a float para formato exacto del nRF53
    float accel_x = acc[0].val1 + (acc[0].val2 / 1000000.0f);
    float accel_y = acc[1].val1 + (acc[1].val2 / 1000000.0f);
    float accel_z = acc[2].val1 + (acc[2].val2 / 1000000.0f);

    float gyro_x = gyr[0].val1 + (gyr[0].val2 / 1000000.0f);
    float gyro_y = gyr[1].val1 + (gyr[1].val2 / 1000000.0f);
    float gyro_z = gyr[2].val1 + (gyr[2].val2 / 1000000.0f);

    // FORMATO EXACTO que espera el nRF53 parser (con float %.3f)
    LOG_INF("-> BMI270 ACCEL: X=%.3f, Y=%.3f, Z=%.3f m/s²",
            (double)accel_x, (double)accel_y, (double)accel_z);
    LOG_INF("-> BMI270 GYRO: X=%.3f, Y=%.3f, Z=%.3f rad/s",
            (double)gyro_x, (double)gyro_y, (double)gyro_z);

    // Publicar datos del BMI270 por MQTT
    mqtt_publish_bmi270(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
}

/**
 * @brief Función OPTIMIZADA para leer BMI270 - FORMATO EXACTO para nRF53
 * Envía los datos en el formato exacto que espera el parser del nRF53
 * OPTIMIZACIÓN: Líneas consecutivas para parsing rápido en nRF53
 */
void get_data_bmi270_fast()
{
    // Lectura directa sin verificaciones - MÁS RÁPIDA
    sensor_sample_fetch(bmi270_dev);
    sensor_channel_get(bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    sensor_channel_get(bmi270_dev, SENSOR_CHAN_GYRO_XYZ, gyr);

    // Convertir valores sensor_value a float para formato exacto del nRF53
    float accel_x = acc[0].val1 + (acc[0].val2 / 1000000.0f);
    float accel_y = acc[1].val1 + (acc[1].val2 / 1000000.0f);
    float accel_z = acc[2].val1 + (acc[2].val2 / 1000000.0f);

    float gyro_x = gyr[0].val1 + (gyr[0].val2 / 1000000.0f);
    float gyro_y = gyr[1].val1 + (gyr[1].val2 / 1000000.0f);
    float gyro_z = gyr[2].val1 + (gyr[2].val2 / 1000000.0f);

    // FORMATO EXACTO que espera el nRF53 parser (con float %.3f)
    // Enviar AMBAS líneas consecutivamente para parsing rápido en nRF53
    LOG_INF("-> BMI270 ACCEL: X=%.3f, Y=%.3f, Z=%.3f m/s²",
            (double)accel_x, (double)accel_y, (double)accel_z);
    LOG_INF("-> BMI270 GYRO: X=%.3f, Y=%.3f, Z=%.3f rad/s",
            (double)gyro_x, (double)gyro_y, (double)gyro_z);
}

/**
 * @brief Timer handler para BME680 - Se ejecuta cada 2 segundos automáticamente
 */
static void bme680_timer_handler(struct k_timer *timer)
{
    // Enviar trabajo al work queue para no bloquear el timer IRQ
    k_work_submit(&bme680_realtime_work);
}

/**
 * @brief Timer handler - Se ejecuta cada 500ms cuando está en modo tiempo real
 * Este método es MÁS EFICIENTE que usar polling en el loop principal
 */
static void bmi270_timer_handler(struct k_timer *timer)
{
    // Enviar trabajo al work queue para no bloquear el timer IRQ
    k_work_submit(&bmi270_realtime_work);
}

/**
 * @brief Work handler para el timer del BMI270 - MÉTODO MÁS EFICIENTE
 */
static void bmi270_realtime_work_handler(struct k_work *work)
{
    // Usar la función rápida para máxima eficiencia
    get_data_bmi270_fast();
}

/**
 * @brief Iniciar modo tiempo real EFICIENTE del BMI270
 * COMPATIBLE con formato exacto del nRF53 parser
 */
static void start_bmi270_realtime(void)
{
    if (bmi270_is_working >= 1)
    {
        bmi270_is_working = 2;
        LOG_INF("-> nrf91: 🚀 BMI270 EFFICIENT real-time mode activated (500ms timer)");
        LOG_INF("-> nrf91: 📡 Using EXACT format for nRF53 BLE integration");

        // Iniciar timer periódico - MÁS EFICIENTE que polling
        k_timer_start(&bmi270_timer, K_MSEC(500), K_MSEC(500));

        // Enviar primer dato inmediatamente en formato correcto
        get_data_bmi270_fast();
    }
    else
    {
        LOG_ERR("-> nrf91: ❌ BMI270 not initialized");
    }
}

/**
 * @brief Detener modo tiempo real EFICIENTE del BMI270
 */
static void stop_bmi270_realtime(void)
{
    if (bmi270_is_working == 2)
    {
        // Detener timer - LIBERA RECURSOS
        k_timer_stop(&bmi270_timer);
        bmi270_is_working = 1;
        LOG_INF("-> nrf91: 🛑 BMI270 efficient real-time mode stopped - timer disabled");
    }
    else
    {
        LOG_WRN("-> nrf91: ⚠️ BMI270 was not in real-time mode");
    }
}

void mqtt_publish_bme680(double temperature, double humidity, double pressure, double gas_resistance) {
    char payload[128];
    int len = snprintf(payload, sizeof(payload),
        "BME680: T=%.2fC H=%.1f%% P=%.1fkPa G=%.1fOhm",
        temperature, humidity, pressure, gas_resistance);
    if (len > 0) {
        struct mqtt_publish_param param;
        param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
        param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC_BME680;
        param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC_BME680);
        param.message.payload.data = (uint8_t *)payload;
        param.message.payload.len = len;
        uint32_t msg_id = sys_rand32_get();
        param.message_id = msg_id;
        param.dup_flag = 0;
        param.retain_flag = 0;
        data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE, (uint8_t *)payload, len, CONFIG_MQTT_PUB_TOPIC_BME680);
    }
}

void mqtt_publish_bmi270(float ax, float ay, float az, float gx, float gy, float gz) {
    char payload[128];
    int len = snprintf(payload, sizeof(payload),
        "BMI270: AX=%.3f AY=%.3f AZ=%.3f GX=%.3f GY=%.3f GZ=%.3f",
        ax, ay, az, gx, gy, gz);
    if (len > 0) {
        struct mqtt_publish_param param;
        param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
        param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC_BMI270;
        param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC_BMI270);
        param.message.payload.data = (uint8_t *)payload;
        param.message.payload.len = len;
        uint32_t msg_id2 = sys_rand32_get();
        param.message_id = msg_id2;
        param.dup_flag = 0;
        param.retain_flag = 0;
        data_publish(&client, MQTT_QOS_1_AT_LEAST_ONCE, (uint8_t *)payload, len, CONFIG_MQTT_PUB_TOPIC_BMI270);
    }
}

/* Stack y datos para hilo GPS */
// #define GPS_THREAD_STACK_SIZE 4096
// static K_THREAD_STACK_DEFINE(gps_stack, GPS_THREAD_STACK_SIZE);
// static struct k_thread gps_thread_data;

// void gps_thread_fn(void *a, void *b, void *c) {
//     while (1) {
//         if (gnss_init_and_start() == 0) {
//             if (server_resolve() == 0 && server_connect() == 0) {
//                 client_post_send();
//             }
//         }
//         k_sleep(K_SECONDS(2));
//     }
// }

int main(void)
{
    LOG_INF("################################################");
    LOG_INF("-> nrf91: 🚀 nRF9151 REAL-TIME Sensor Monitoring Starting");
    LOG_INF("-> nrf91: 📋 Protocol: BINARY (0x01=ON, 0x00=OFF) or TEXT commands");
    LOG_INF("-> nrf91: 🌡️ BME680: AUTO every 2 seconds | 🏃 BMI270: AUTO every 500ms");
    LOG_INF("-> nrf91: ⚡ OPTIMIZED: Both sensors use efficient timers (no polling)");
    LOG_INF("-> nrf91: 🎯 AUTO-START: Both sensors start automatically at boot");
    LOG_INF("################################################");

    // Inicializar work queues para procesamiento de sensores
    k_work_init(&bme_work, bme_work_handler);
    k_work_init(&bmi_work, bmi_work_handler);

    // Inicializar timer y work para BMI270 EFICIENTE (500ms)
    k_timer_init(&bmi270_timer, bmi270_timer_handler, NULL);
    k_work_init(&bmi270_realtime_work, bmi270_realtime_work_handler);

    // Inicializar timer y work para BME680 automático (2 segundos)
    k_timer_init(&bme680_timer, bme680_timer_handler, NULL);
    k_work_init(&bme680_realtime_work, bme680_realtime_work_handler);

    LOG_INF("-> nrf91: ✅ Work queues and timers initialized (BMI270: 500ms, BME680: 2s)");

    // Usar la variable global bme680_dev

    // Configurar LED
    if (!gpio_is_ready_dt(&led))
    {
        LOG_ERR("-> nrf91: ❌ LED GPIO not ready");
        return -1;
    }

    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE); // Configurar como salida, inicialmente inactiva
    if (ret < 0)
    {
        LOG_ERR("-> nrf91: ❌ LED configure failed: %d", ret);
        return ret;
    }

    // LED apagado por defecto
    control_led(false);
    LOG_INF("-> nrf91: ✅ LED initialized OFF");

    // Configurar UART
    if (!device_is_ready(uart_dev))
    {
        LOG_ERR("-> nrf91: ❌ UART not ready");
        return -1;
    }

    ret = uart_irq_callback_user_data_set(uart_dev, uart_callback, NULL);
    if (ret < 0)
    {
        LOG_ERR("-> nrf91: ❌ UART callback failed: %d", ret);
        return ret;
    }

    // * MQTT y conexión LTE
    LOG_INF("-> nrf91: 📡 Initializing MQTT client and LTE connection...");
    int err;
    uint32_t connect_attempt = 0;

    if (dk_leds_init() != 0)
    {
        LOG_ERR("Failed to initialize the LED library");
    }

    err = modem_configure();
    if (err)
    {
        LOG_ERR("Failed to configure the modem");
        return 0;
    }

    if (dk_buttons_init(button_handler) != 0)
    {
        LOG_ERR("Failed to initialize the buttons library");
    }

    err = client_init(&client);
    if (err)
    {
        LOG_ERR("Failed to initialize MQTT client: %d", err);
        return 0;
    }

    // * MQTT y conexión LTE

    uart_irq_rx_enable(uart_dev);
    LOG_INF("-> nrf91: ✅ UART ready for nRF5340 commands");

    LOG_INF("################################################");
    LOG_INF("-> nrf91: 🎯 READY! Real-time sensor monitoring ACTIVE!");
    LOG_INF("-> nrf91: 👂 Listening for BINARY (0x01/0x00) or TEXT commands");
    LOG_INF("-> nrf91: 🌡️ BME680: Auto-sending data every 2 seconds");
    LOG_INF("-> nrf91: 🏃 BMI270: Auto-sending data every 500ms (real-time)");
    LOG_INF("-> nrf91: 📋 Commands: LED_ON/OFF, BMI_STOP (stop BMI270)");
    LOG_INF("################################################");

    // Configurar BME680 sensor
    start_bme680(bme680_dev);

    // Configurar BMI270 sensor
    bmi270_is_working = start_bmi270(); // Corregido: BMI270, no BME270
    if (bmi270_is_working <= 0)
    {
        LOG_ERR("-> nrf91: ❌ Failed to start BMI270 sensor");
    }
    else
    {
        LOG_INF("-> nrf91: ✅ BMI270 sensor initialized successfully");
        // Iniciar automáticamente el modo tiempo real del BMI270
        bmi270_is_working = 2; // Forzar modo tiempo real
        k_timer_start(&bmi270_timer, K_MSEC(500), K_MSEC(500));
        LOG_INF("-> nrf91: 🚀 BMI270 real-time mode STARTED automáticamente (500ms)");
    }

    // Iniciar automáticamente el timer del BME680 (2 segundos)
    k_timer_start(&bme680_timer, K_SECONDS(2), K_SECONDS(2));
    LOG_INF("-> nrf91: 🌡️ BME680 automatic timer STARTED (2 seconds)");
    LOG_INF("-> nrf91: ✅ Both sensors running in real-time mode automáticamente!");

    int mqtt_connected = 0;

    // MQTT reconnection loop robusto y correcto
    do_connect:
    if (connect_attempt++ > 0) {
        LOG_INF("Reconnecting in %d seconds...", CONFIG_MQTT_RECONNECT_DELAY_S);
        k_sleep(K_SECONDS(CONFIG_MQTT_RECONNECT_DELAY_S));
    }
    err = mqtt_connect(&client);
    if (err) {
        LOG_ERR("Error in mqtt_connect: %d", err);
        goto do_connect;
    }
    mqtt_connected = 1;

    err = fds_init(&client, &fds);
    if (err) {
        LOG_ERR("Error in fds_init: %d", err);
        mqtt_connected = 0;
        goto do_connect;
    }

    // // Lanzar el hilo GPS/CoAP
    // k_thread_create(&gps_thread_data, gps_stack, GPS_THREAD_STACK_SIZE,
    //                 gps_thread_fn, NULL, NULL, NULL,
    //                 7, 0, K_NO_WAIT);
    // LOG_INF("GPS/CoAP periodic thread started (every 2s)");

    // Loop principal: solo desconectar si está conectado y hay error
    while (mqtt_connected) {
        int result = mqtt_loop();
        if (result != 0) {
            LOG_ERR("Error in mqtt_loop: %d", result);
            LOG_INF("Disconnecting MQTT client");
            err = mqtt_disconnect(&client);
            if (err) {
                LOG_ERR("Could not disconnect MQTT client: %d", err);
            }
            mqtt_connected = 0;
            goto do_connect;
        }
        k_sleep(K_MSEC(500));
    }

    if (mqtt_connected) {
        LOG_INF("Disconnecting MQTT client");
        err = mqtt_disconnect(&client);
        if (err) {
            LOG_ERR("Could not disconnect MQTT client: %d", err);
        }
        mqtt_connected = 0;
    }
    goto do_connect;

    // Loop principal OPTIMIZADO - Ambos sensores usan timers eficientes
    while (1) {
        if (mqtt_connected) {
            int result = mqtt_loop();
            if (result != 0) {
                LOG_ERR("Error in mqtt_loop: %d", result);
                break;
            }
        }
        k_sleep(K_MSEC(5000));
    }

    if (mqtt_connected) {
        LOG_INF("Disconnecting MQTT client");
        err = mqtt_disconnect(&client);
        if (err) {
            LOG_ERR("Could not disconnect MQTT client: %d", err);
        }
        mqtt_connected = 0;
    }
    goto do_connect;

    // Loop principal OPTIMIZADO - Ambos sensores usan timers eficientes
    while (1)
    {
        if (connected == 1)
        {
            int result = mqtt_loop();
            if (result != 0)
            {
                LOG_ERR("Error in mqtt_loop: %d", result);
                break;
            }
        }
        // Los sensores se manejan automáticamente con sus timers
        // BME680: timer cada 2 segundos
        // BMI270: timer cada 500ms
        // El loop principal solo supervisa el estado del sistema

        k_sleep(K_MSEC(5000)); // Sleep más largo ya que los sensores son automáticos
        // LOG_INF("-> nrf91: 💓 System alive - LED: %s | BMI270: %s | BME680: %s",
        //         led_state ? "ON" : "OFF",
        //         bmi270_is_working == 2 ? "ACTIVE-500ms" : "STOPPED",
        //         "ACTIVE-2s");
    }

    return 0;
}
