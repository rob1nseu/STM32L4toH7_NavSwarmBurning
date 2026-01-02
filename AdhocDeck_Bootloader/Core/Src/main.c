/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file : main.c
  * @brief : Bootloader for STM32H7 - Firmware Update & APP Jump
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 状态定义
typedef enum {
    STATE_SEND = 0x0,     // 发送状态
    STATE_RECEIVE = 0x1,  // 接收状态
    STATE_COPY = 0x2,     // 复制状态
    STATE_SKIP = 0x3      // 跳过状态
} UpdateState;

// 版本号和状态信息
typedef struct {
    uint32_t app_version;         // APP当前版本号
    uint32_t download_version;    // 下载区版本号
    uint32_t update_state;        // 更新状态（低2位有效）
    uint32_t reserved;            // 保留字段
} VersionInfo;

// 下载区头部信息
typedef struct {
    uint32_t data_size;   // 需要复制的数据长度（字节）
    uint32_t crc32;       // 数据的CRC32校验值
    uint32_t reserved1;   // 保留字段
    uint32_t reserved2;
} DownloadHeader;

// CRC32计算（用于校验APP/版本号）
#define CRC32_POLY 0xEDB88320
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ADDR           (0x08020000U)      // APP起始地址，128KB处
#define DOWNLOAD_ADDR      (0x081E0000U)      // 下载区起始地址，Bank2后128KB
#define VERSION_ADDR       (0x0801FFF0U)      // 版本号存储地址，Bootloader区最后16字节

// H743 Flash参数
//#define FLASH_SECTOR_SIZE  (128U * 1024U)     // 每个扇区128KB
#define MAX_APP_SIZE       (1920U * 1024U) // 最大1920KB(限制复制文件大小)

// LED引脚配置
#define BLINK_GPIO_PORT    GPIOA
#define BLINK_GPIO_PIN     GPIO_PIN_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
// 函数声明
static uint32_t calc_crc32(const uint8_t *data, uint32_t len);
static HAL_StatusTypeDef read_version_info(VersionInfo *info);
static HAL_StatusTypeDef update_version_info(VersionInfo *info);
static HAL_StatusTypeDef copy_download_to_app(void);
static UpdateState get_update_state(void);
static void JumpToApp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief 计算CRC32校验和
  */
static uint32_t calc_crc32(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1) ? CRC32_POLY : 0);
        }
    }
    return ~crc;
}

/**
  * @brief 获取更新状态
  */
static UpdateState get_update_state(void) {
    VersionInfo info;
    if (read_version_info(&info) == HAL_OK) {
        return (UpdateState)(info.update_state & 0x03); // 读取失败时默认跳过
    }
    return STATE_SKIP;
}

/**
  * @brief 读取版本号和状态信息
  */
static HAL_StatusTypeDef read_version_info(VersionInfo *info) {
    if (info == NULL) return HAL_ERROR;
    memcpy(info, (void*)VERSION_ADDR, sizeof(VersionInfo));
    return HAL_OK;
}

/**
  * @brief 更新版本号和状态信息（需修改，需要添加“修改后状态”的形参）
  */
static HAL_StatusTypeDef update_version_info(VersionInfo *info) {
    if (info == NULL) return HAL_ERROR;

    // 确保状态只使用低2位
    info->update_state &= 0x03;

    // 解锁Flash
    if (HAL_FLASH_Unlock() != HAL_OK) return HAL_ERROR;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    // 擦除版本号所在扇区（扇区0）
    FLASH_EraseInitTypeDef erase_init = {0};
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Sector = 0;  // 扇区0
    erase_init.NbSectors = 1;

    uint32_t sector_error = 0;
    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // 写入版本号（按256位字写入，H7需要对齐到32字节）
    // 将VersionInfo转换为256位对齐的数据
    uint64_t temp_data[4] = {0}; // 4*8=32字节
    memcpy(temp_data, info, sizeof(VersionInfo));

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, VERSION_ADDR, (uint32_t)temp_data) != HAL_OK) {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

/**
  * @brief 从下载区复制到APP区
  */
static HAL_StatusTypeDef copy_download_to_app(void) {
    // 读取下载区头部信息
    DownloadHeader header;
    memcpy(&header, (void*)DOWNLOAD_ADDR, sizeof(DownloadHeader));

    // 验证数据长度
    if (header.data_size == 0 || header.data_size > MAX_APP_SIZE) {
        return HAL_ERROR;
    }

    // 计算有效数据起始地址
    uint32_t data_start_addr = DOWNLOAD_ADDR + sizeof(DownloadHeader);

    // 解锁Flash
    if (HAL_FLASH_Unlock() != HAL_OK) return HAL_ERROR;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

    // 计算需要擦除的扇区数
    uint32_t app_sector_start = (APP_ADDR - FLASH_BASE) / FLASH_SECTOR_SIZE;
    uint32_t data_size_aligned = (header.data_size + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);
    uint32_t sectors_needed = data_size_aligned / FLASH_SECTOR_SIZE;

    // 擦除APP区域
    FLASH_EraseInitTypeDef erase_init = {0};
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Sector = app_sector_start;
    erase_init.NbSectors = sectors_needed;

    uint32_t sector_error = 0;
    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // 逐32字节（256位）复制
    uint32_t flashword_size = 32; // H7的FLASHWORD是256位=32字节
    for (uint32_t offset = 0; offset < header.data_size; offset += flashword_size) {
        uint32_t remaining = header.data_size - offset;
        uint32_t copy_size = (remaining >= flashword_size) ? flashword_size : remaining;

        // 准备数据（需要32字节对齐）
        uint8_t temp_buffer[32] = {0};
        memcpy(temp_buffer, (void*)(data_start_addr + offset), copy_size);

        // 编程FLASHWORD
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                             APP_ADDR + offset,
                             (uint32_t)temp_buffer) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }

    // 验证复制数据的CRC（可选，可以注释掉）
    // uint32_t calc_crc = calc_crc32((uint8_t*)APP_ADDR, header.data_size);
    // if (calc_crc != header.crc32) {
    //     HAL_FLASH_Lock();
    //     return HAL_ERROR;
    // }

    HAL_FLASH_Lock();
    return HAL_OK;
}

/**
  * @brief 跳转到APP区
  */
static void JumpToApp(void) {
    typedef void (*pFunction)(void);
    pFunction JumpToApplication;

    __disable_irq(); // 关中断

    // 读取APP复位入口地址
    uint32_t appResetHandlerAddr = *(__IO uint32_t*)(APP_ADDR + 4);
    JumpToApplication = (pFunction)appResetHandlerAddr;

    // 设置APP栈顶 + 重定向向量表
    __set_MSP(*(__IO uint32_t*)APP_ADDR);
    SCB->VTOR = APP_ADDR;

    // 重新使能中断（APP会重新初始化中断）
    __enable_irq();

    JumpToApplication();
}
/* USER CODE END 0 */

/**
  * @brief The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    MPU_Config();

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();  // ⚠️ 必须初始化PA1为输出！

    /* USER CODE BEGIN 2 */
    // Bootloader启动指示：快闪3秒
    for (volatile uint32_t i = 0; i < 15; i++) {
        HAL_GPIO_TogglePin(BLINK_GPIO_PORT, BLINK_GPIO_PIN);
        HAL_Delay(200);
    }
    HAL_GPIO_WritePin(BLINK_GPIO_PORT, BLINK_GPIO_PIN, GPIO_PIN_RESET);

    // 读取版本信息和状态
    VersionInfo ver_info;
    HAL_StatusTypeDef ret = read_version_info(&ver_info);

    // 获取当前更新状态
    UpdateState current_state = get_update_state();

    // 根据状态执行不同操作
    switch (current_state) {
        case STATE_SEND:
            // 发送状态处理，待添加发送当前版本信息的代码
            break;

        case STATE_RECEIVE:
            // 接收状态处理，待接收新固件的代码
            break;

        case STATE_COPY:
            // 复制状态：执行固件更新
            if (ret == HAL_OK && ver_info.download_version != ver_info.app_version) {
                HAL_GPIO_WritePin(BLINK_GPIO_PORT, BLINK_GPIO_PIN, GPIO_PIN_SET); // 点亮LED表示升级中

                // 从下载区复制到APP区
                if (copy_download_to_app() == HAL_OK) {
                    // 升级成功：更新APP版本号为下载区版本号，并将状态设为跳过
                    ver_info.app_version = ver_info.download_version;
                    ver_info.update_state = STATE_SKIP;
                    update_version_info(&ver_info);
                }

                HAL_GPIO_WritePin(BLINK_GPIO_PORT, BLINK_GPIO_PIN, GPIO_PIN_RESET);
            }
            break;

        case STATE_SKIP:
        default:
            // 跳过状态：直接跳转到APP
            break;
    }

    // 跳转到APP
    JumpToApp();

    /* USER CODE END 2 */

    while (1) {
        // 正常情况下不会执行到这里
        // 如果跳转失败，这里会闪烁LED
        HAL_GPIO_WritePin(BLINK_GPIO_PORT, BLINK_GPIO_PIN, GPIO_PIN_SET);
        HAL_Delay(500);
        HAL_GPIO_WritePin(BLINK_GPIO_PORT, BLINK_GPIO_PIN, GPIO_PIN_RESET);
        HAL_Delay(500);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}
/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
