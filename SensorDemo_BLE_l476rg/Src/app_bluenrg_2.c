/**
  ******************************************************************************
  * File Name          : app_bluenrg_2.c
  * Description        : Implémentation des fonctions spécifiques à la stack BLE et aux capteurs.
  ******************************************************************************
  * (C) Copyright 2023 STMicroelectronics.
  * Licensed under MCD-ST Liberty SW License Agreement V2.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_bluenrg_2.h"
#include <stdlib.h>
#include "sensor.h"
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_events.h"
#include "hci_tl.h"
#include "gatt_db.h"
#include "bluenrg_utils.h"
#include "stm32l4xx_nucleo.h"
#include "lps22hh_reg.h"
#include "hts221_reg.h"
#include "sensor_platform.h"

/* USER CODE BEGIN Includes */
typedef struct {
    float_t x0;
    float_t y0;
    float_t x1;
    float_t y1;
} lin_t;
/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
#define USE_BUTTON (0)  // 1: envoi manuel via bouton, 0: envoi automatique

/* Private variables ---------------------------------------------------------*/
extern AxesRaw_t x_axes;
extern AxesRaw_t g_axes;
extern AxesRaw_t m_axes;
extern AxesRaw_t q_axes;

extern __IO uint16_t connection_handle;
extern volatile uint8_t set_connectable;
extern volatile int connected;
uint8_t bdaddr[BDADDR_SIZE];
static volatile uint8_t user_button_init_state = 1;
static volatile uint8_t user_button_pressed = 0;

/* Private function prototypes -----------------------------------------------*/
static void User_Process(void);
static void User_Init(void);
static uint8_t Sensor_DeviceInit(void);
static void Sensor_Init(void);
void Read_Real_Environmental_Values(float *pressure, float *temp, float *humidity);
static void Set_Random_Motion_Values(uint32_t cnt);
static void Reset_Motion_Values(void);

/* Initialisation de la stack BLE et des capteurs */
void MX_BlueNRG_2_Init(void)
{
  uint8_t ret;
  User_Init();
  user_button_init_state = BSP_PB_GetState(BUTTON_KEY);
  hci_init(APP_UserEvtRx, NULL);
  PRINT_DBG("BlueNRG-2 SensorDemo_BLESensor-App Application\r\n");

  ret = Sensor_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS)
  {
    BSP_LED_On(LED2);
    PRINT_DBG("Sensor_DeviceInit() échoué (0x%02x)\r\n", ret);
    while (1);
  }
  PRINT_DBG("BLE Stack et configuration du device initialisés\r\n");
}

/* Boucle de traitement principal */
void MX_BlueNRG_2_Process(void)
{
  hci_user_evt_proc();
  User_Process();
}

/* Initialisation des composants utilisateur */
static void User_Init(void)
{
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  BSP_LED_Init(LED2);
  BSP_COM_Init(COM1);
}

/* Initialisation du device capteur et configuration BLE */
uint8_t Sensor_DeviceInit(void)
{
  uint8_t ret;
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  uint8_t device_name[] = {SENSOR_DEMO_NAME};
  uint8_t hwVersion;
  uint16_t fwVersion;
  uint8_t bdaddr_len_out;
  uint8_t config_data_stored_static_random_address = 0x80;

  /* Réinitialisation du dispositif */
  hci_reset();
  HAL_Delay(2000);
  getBlueNRGVersion(&hwVersion, &fwVersion);
  PRINT_DBG("HWver %d, FWver %d\r\n", hwVersion, fwVersion);

  ret = aci_hal_read_config_data(config_data_stored_static_random_address,
                                 &bdaddr_len_out, bdaddr);
  if (ret)
    PRINT_DBG("Échec de lecture de l’adresse statique\r\n");

  if ((bdaddr[5] & 0xC0) != 0xC0)
  {
    PRINT_DBG("Adresse statique mal formée\r\n");
    while (1);
  }

  ret = aci_hal_set_tx_power_level(1, 4);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("Erreur aci_hal_set_tx_power_level() 0x%04x\r\n", ret);
    return ret;
  }
  PRINT_DBG("Tx power level configuré\r\n");

  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_init() échoué: 0x%02x\r\n", ret);
    return ret;
  }
  PRINT_DBG("aci_gatt_init() OK\r\n");

  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x07,
                     &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gap_init() échoué: 0x%02x\r\n", ret);
    return ret;
  }
  PRINT_DBG("aci_gap_init() OK\r\n");

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, sizeof(device_name), device_name);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("aci_gatt_update_char_value() échoué: 0x%02x\r\n", ret);
    return ret;
  }
  PRINT_DBG("Nom du device mis à jour\r\n");

  ret = aci_gap_set_authentication_requirement(BONDING, MITM_PROTECTION_REQUIRED, SC_IS_SUPPORTED,
                                                 KEYPRESS_IS_NOT_SUPPORTED, 7, 16,
                                                 USE_FIXED_PIN_FOR_PAIRING, 123456, 0x00);
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("Erreur authentication requirement: 0x%02x\r\n", ret);
    return ret;
  }
  PRINT_DBG("Authentification configurée\r\n");

  PRINT_DBG("Stack BLE initialisée\r\n");

  ret = Add_HWServW2ST_Service();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("Erreur ajout service HW: 0x%02x\r\n", ret);
    while (1);
  }
  PRINT_DBG("Service HW ajouté\r\n");

  ret = Add_SWServW2ST_Service();
  if (ret != BLE_STATUS_SUCCESS)
  {
    PRINT_DBG("Erreur ajout service SW: 0x%02x\r\n", ret);
    while (1);
  }
  PRINT_DBG("Service SW ajouté\r\n");

  return BLE_STATUS_SUCCESS;
}

void User_Process(void)
{
  float data_t, data_p, data_h;
  static uint32_t counter = 0;

  if (set_connectable)
  {
    Set_DeviceConnectable();
    set_connectable = FALSE;
  }

  BSP_LED_Toggle(LED2);

  if (connected)
  {
    Read_Real_Environmental_Values(&data_p, &data_t, &data_h);
    Environmental_Update((int32_t)(data_p * 100), (int16_t)(data_t * 10), (int16_t)(data_h * 10));

    Set_Random_Motion_Values(counter);
    Acc_Update(&x_axes, &g_axes, &m_axes);
    Quat_Update(&q_axes);

    counter = (counter + 1) % 40;
    if (counter == 0)
      Reset_Motion_Values();

    HAL_Delay(1000);
  }
}

static void Sensor_Init(void)
{
    stmdev_ctx_t lps22_ctx;
    uint8_t whoamI, rst;

    lps22_ctx.write_reg = platform_write_lps22;
    lps22_ctx.read_reg  = platform_read_lps22;
    lps22_ctx.mdelay    = platform_delay;
    lps22_ctx.handle    = &SENSOR_BUS;

    platform_delay(5);
    lps22hh_device_id_get(&lps22_ctx, &whoamI);
    if (whoamI != LPS22HH_ID)
    {
        PRINT_DBG("LPS22HH non détecté\r\n");
        while(1);
    }
    lps22hh_reset_set(&lps22_ctx, PROPERTY_ENABLE);
    do {
         lps22hh_reset_get(&lps22_ctx, &rst);
    } while(rst);
    lps22hh_block_data_update_set(&lps22_ctx, PROPERTY_ENABLE);
    lps22hh_data_rate_set(&lps22_ctx, LPS22HH_10_Hz_LOW_NOISE);

    stmdev_ctx_t hts221_ctx;
    uint8_t hts_id;
    hts221_ctx.write_reg = platform_write_hts221;
    hts221_ctx.read_reg  = platform_read_hts221;
    hts221_ctx.mdelay    = platform_delay;
    hts221_ctx.handle    = &SENSOR_BUS;

    hts221_device_id_get(&hts221_ctx, &hts_id);
    if (hts_id != HTS221_ID)
    {
        PRINT_DBG("HTS221 non détecté\r\n");
        while(1);
    }
    hts221_block_data_update_set(&hts221_ctx, PROPERTY_ENABLE);
    hts221_data_rate_set(&hts221_ctx, HTS221_ODR_1Hz);
    hts221_power_on_set(&hts221_ctx, PROPERTY_ENABLE);
}

void Read_Real_Environmental_Values(float *pressure, float *temp, float *humidity)
{
    static uint8_t sensors_initialized = 0;
    if (!sensors_initialized)
    {
         Sensor_Init();
         sensors_initialized = 1;
    }

    stmdev_ctx_t lps22_ctx, hts221_ctx;
    lps22_ctx.write_reg = platform_write_lps22;
    lps22_ctx.read_reg  = platform_read_lps22;
    lps22_ctx.mdelay    = platform_delay;
    lps22_ctx.handle    = &SENSOR_BUS;

    uint32_t data_raw_pressure = 0;
    int16_t data_raw_temperature = 0;
    lps22hh_status_t status;
    lps22hh_read_reg(&lps22_ctx, LPS22HH_STATUS, (uint8_t *)&status, 1);
    if (status.p_da)
    {
         lps22hh_pressure_raw_get(&lps22_ctx, &data_raw_pressure);
         *pressure = lps22hh_from_lsb_to_hpa(data_raw_pressure);
    }
    else
         *pressure = 0;

    if (status.t_da)
    {
         lps22hh_temperature_raw_get(&lps22_ctx, &data_raw_temperature);
         *temp = lps22hh_from_lsb_to_celsius(data_raw_temperature);
    }
    else
         *temp = 0;

    hts221_ctx.write_reg = platform_write_hts221;
    hts221_ctx.read_reg  = platform_read_hts221;
    hts221_ctx.mdelay    = platform_delay;
    hts221_ctx.handle    = &SENSOR_BUS;

    lin_t lin_hum;
    hts221_hum_adc_point_0_get(&hts221_ctx, &lin_hum.x0);
    hts221_hum_rh_point_0_get(&hts221_ctx, &lin_hum.y0);
    hts221_hum_adc_point_1_get(&hts221_ctx, &lin_hum.x1);
    hts221_hum_rh_point_1_get(&hts221_ctx, &lin_hum.y1);

    PRINT_DBG("Calibration Humidité: x0=%.2f, y0=%.2f, x1=%.2f, y1=%.2f\r\n",
               lin_hum.x0, lin_hum.y0, lin_hum.x1, lin_hum.y1);

    int16_t data_raw_humidity = 0;
    hts221_status_reg_t hts_status;
    hts221_status_get(&hts221_ctx, &hts_status);
    PRINT_DBG("HTS221 status: h_da=%d\r\n", hts_status.h_da);

    if (hts_status.h_da)
    {
         hts221_humidity_raw_get(&hts221_ctx, &data_raw_humidity);
         PRINT_DBG("Raw humidity: %d\r\n", data_raw_humidity);
         *humidity = ((lin_hum.y1 - lin_hum.y0) * data_raw_humidity +
                      ((lin_hum.x1 * lin_hum.y0) - (lin_hum.x0 * lin_hum.y1))) /
                     (lin_hum.x1 - lin_hum.x0);
         if (*humidity < 0) *humidity = 0;
         if (*humidity > 100) *humidity = 100;
    }
    else
         *humidity = 0;
}

static void Set_Random_Motion_Values(uint32_t cnt)
{
  if (cnt < 20)
  {
    x_axes.AXIS_X +=  10  + ((uint64_t)rand() * 3  * cnt) / RAND_MAX;
    x_axes.AXIS_Y += -10  - ((uint64_t)rand() * 5  * cnt) / RAND_MAX;
    x_axes.AXIS_Z +=  10  + ((uint64_t)rand() * 7  * cnt) / RAND_MAX;
    g_axes.AXIS_X +=  100 + ((uint64_t)rand() * 2  * cnt) / RAND_MAX;
    g_axes.AXIS_Y += -100 - ((uint64_t)rand() * 4  * cnt) / RAND_MAX;
    g_axes.AXIS_Z +=  100 + ((uint64_t)rand() * 6  * cnt) / RAND_MAX;
    m_axes.AXIS_X +=  3   + ((uint64_t)rand() * 3  * cnt) / RAND_MAX;
    m_axes.AXIS_Y += -3   - ((uint64_t)rand() * 4  * cnt) / RAND_MAX;
    m_axes.AXIS_Z +=  3   + ((uint64_t)rand() * 5  * cnt) / RAND_MAX;
    q_axes.AXIS_X -= 100 + ((uint64_t)rand() * 3  * cnt) / RAND_MAX;
    q_axes.AXIS_Y += 100 + ((uint64_t)rand() * 5  * cnt) / RAND_MAX;
    q_axes.AXIS_Z -= 100 + ((uint64_t)rand() * 7  * cnt) / RAND_MAX;
  }
  else
  {
    x_axes.AXIS_X += -10  - ((uint64_t)rand() * 3  * cnt) / RAND_MAX;
    x_axes.AXIS_Y +=  10  + ((uint64_t)rand() * 5  * cnt) / RAND_MAX;
    x_axes.AXIS_Z += -10  - ((uint64_t)rand() * 7  * cnt) / RAND_MAX;
    g_axes.AXIS_X += -100 - ((uint64_t)rand() * 2  * cnt) / RAND_MAX;
    g_axes.AXIS_Y +=  100 + ((uint64_t)rand() * 4  * cnt) / RAND_MAX;
    g_axes.AXIS_Z += -100 - ((uint64_t)rand() * 6  * cnt) / RAND_MAX;
    m_axes.AXIS_X += -3   - ((uint64_t)rand() * 7  * cnt) / RAND_MAX;
    m_axes.AXIS_Y +=  3   + ((uint64_t)rand() * 9  * cnt) / RAND_MAX;
    m_axes.AXIS_Z += -3   - ((uint64_t)rand() * 3  * cnt) / RAND_MAX;
    q_axes.AXIS_X += 200 + ((uint64_t)rand() * 7  * cnt) / RAND_MAX;
    q_axes.AXIS_Y -= 150 + ((uint64_t)rand() * 3  * cnt) / RAND_MAX;
    q_axes.AXIS_Z += 10  + ((uint64_t)rand() * 5  * cnt) / RAND_MAX;
  }
}

/* Réinitialisation des valeurs de mouvement */
static void Reset_Motion_Values(void)
{
  x_axes.AXIS_X = (x_axes.AXIS_X % 2000 == 0) ? -x_axes.AXIS_X : 10;
  x_axes.AXIS_Y = (x_axes.AXIS_Y % 2000 == 0) ? -x_axes.AXIS_Y : -10;
  x_axes.AXIS_Z = (x_axes.AXIS_Z % 2000 == 0) ? -x_axes.AXIS_Z : 10;
  g_axes.AXIS_X = (g_axes.AXIS_X % 2000 == 0) ? -g_axes.AXIS_X : 100;
  g_axes.AXIS_Y = (g_axes.AXIS_Y % 2000 == 0) ? -g_axes.AXIS_Y : -100;
  g_axes.AXIS_Z = (g_axes.AXIS_Z % 2000 == 0) ? -g_axes.AXIS_Z : 100;
  m_axes.AXIS_X = (g_axes.AXIS_X % 2000 == 0) ? -m_axes.AXIS_X : 3;
  m_axes.AXIS_Y = (g_axes.AXIS_Y % 2000 == 0) ? -m_axes.AXIS_Y : -3;
  m_axes.AXIS_Z = (g_axes.AXIS_Z % 2000 == 0) ? -m_axes.AXIS_Z : 3;
  q_axes.AXIS_X = -q_axes.AXIS_X;
  q_axes.AXIS_Y = -q_axes.AXIS_Y;
  q_axes.AXIS_Z = -q_axes.AXIS_Z;
}

/* Récupération des versions matérielle et firmware */
uint8_t getBlueNRGVersion(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;

  status = hci_read_local_version_information(&hci_version, &hci_revision, &lmp_pal_version,
                                               &manufacturer_name, &lmp_pal_subversion);
  if (status == BLE_STATUS_SUCCESS)
  {
    *hwVersion = hci_revision >> 8;
    *fwVersion = ((hci_revision & 0xFF) << 8) | (((lmp_pal_subversion >> 4) & 0xF) << 4) | (lmp_pal_subversion & 0xF);
  }
  return status;
}

/* Callback pour le bouton poussoir */
void BSP_PB_Callback(Button_TypeDef Button)
{
  user_button_pressed = 1;
}

/* Callbacks de la pile BLE */
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  connected = TRUE;
  connection_handle = Connection_Handle;
  BSP_LED_Off(LED2);
}

void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
  connected = FALSE;
  set_connectable = TRUE;
  connection_handle = 0;
  PRINT_DBG("Déconnecté\r\n");
  BSP_LED_On(LED2);
}

void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  Read_Request_CB(Attribute_Handle);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
