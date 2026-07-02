# YEmbeddedLib — Embedded Sensor Driver Collection

MCU 嵌入式传感器驱动库，提供多平台可移植的 IMU / 磁力计驱动和抽象数据类型。

## 目录结构

```
YEmbeddedLib/
├── config.h                            # 全局编译期配置 (量程、payload 包头)
├── Interface/                          # 平台无关的数据类型接口
│   ├── geometry/
│   │   └── vector3/
│   │       ├── vector3.h               # Vector3 — 通用三维浮点向量
│   │       └── vector3.c               # (预留向量运算函数)
│   └── sensor/
│       ├── imu/
│       │   ├── imu.h                   # Imu — 6 轴 IMU 数据 + 序列化接口
│       │   └── imu.c                   # Imu2payload 实现 (TODO)
│       ├── mag/
│       │   └── mag.h                   # MagneticField — 磁力计数据 (µT)
│       └── temp/
│           └── temp.h                  # Temperature_Celsius / Fahrenheit / Kelvin
└── Device/                             # 特定传感器驱动
    ├── MPU6500/
    │   ├── mpu6500.h                   # MPU-6500 6-axis IMU 驱动 (handle-based HAL)
    │   ├── mpu6500.c
    │   └── *.pdf                       # 数据手册
    └── IST8310/
        ├── ist8310.h                   # IST8310 3-axis 磁力计驱动 (handle-based HAL)
        ├── ist8310.c
        └── *.pdf                       # 数据手册
```

## 架构设计

三层分离:

| 层 | 目录 | 职责 |
|----|------|------|
| Interface | `Interface/` | 平台无关的数据结构 (Imu、MagneticField、Vector3) |
| Device | `Device/` | 寄存器读写、初始化、数据解析，输出 Interface 类型 |
| Config | `config.h` | 编译期量程选择 (`ACCEL_RANGE`、`GYRO_RANGE`) 和 payload 定义 |

依赖方向: `Device` → `Interface` → `Config`，器件间相互独立。

## HAL 模式

本库采用 **Handle 注入** 模式实现硬件抽象：用户为每个传感器创建一个 handle 结构体，填入指向自己 MCU 平台 I/O 函数的指针，然后传给所有 API 调用。与 `__weak` 弱链接方案相比，支持多实例、易于单元测试。

### MPU6500 使用示例

**1. 配置 `config.h`**

```c
#define ACCEL_RANGE 4    // ±4 g
#define GYRO_RANGE 1000  // ±1000 dps
```

**2. 实现平台 I/O 函数并填充 handle**

```c
#include "mpu6500.h"

// --- 平台相关实现 (以 STM32 HAL 为例) ---
static void     my_delay_ms(uint32_t ms)  { HAL_Delay(ms); }
static uint32_t my_get_stamp(void)        { return HAL_GetTick(); }
static void     my_enable(void)           { /* 拉高 EN 引脚 */ }
static void     my_disable(void)          { /* 拉低 EN 引脚 */ }
static uint8_t  my_read_reg(uint8_t reg)  { /* I2C/SPI 读写 */ return 0; }
static void     my_write_reg(uint8_t r, uint8_t d) { /* ... */ }
static void     my_read_regs(uint8_t s, uint8_t *b, uint8_t l) { /* 阻塞批量读 */ }
static void     my_read_regs_dma(uint8_t s, uint8_t *tx, uint8_t *rx, uint8_t l) { /* DMA 批量读 */ }

MPU6500_handle mpu = {
    .dma_state                 = MPU6500_IDLE,
    .mpu6500_delay_ms          = my_delay_ms,
    .mpu6500_get_stamp_ms      = my_get_stamp,
    .mpu6500_enact             = my_enable,
    .mpu6500_deact             = my_disable,
    .mpu6500_read_reg          = my_read_reg,
    .mpu6500_write_reg         = my_write_reg,
    .mpu6500_read_regs         = my_read_regs,
    .mpu6500_read_regs_dma     = my_read_regs_dma,
};
```

**3. 初始化并使用**

```c
int main(void) {
    Imu imu;
    Temperature_Celsius temp;

    if (MPU6500_Init(&mpu) != 0) {
        // Who-Am-I 校验失败 — 检查接线
        while (1);
    }

    while (1) {
        MPU6500_ReadData(&mpu, &imu, &temp);

        // imu.acceleration.x/y/z      → m/s²
        // imu.angular_velocity.x/y/z  → deg/s
        // temp.data                    → °C
        // imu.timestamp_ms              → ms
    }
}
```

**4. DMA 模式**

```c
// 在主循环中触发非阻塞读
MPU6500_ReadData_DMA(&mpu, &imu, &temp);

// 在 SPI / I2C DMA 完成中断服务函数中调用
void DMA_IRQHandler(void) {
    MPU6500_On_ReadData_DMA_Cplt(&mpu, &imu, &temp);
    // 此时 imu 和 temp 已更新完毕
}
```

### IST8310 使用示例

```c
#include "ist8310.h"

IST8310_handle ist = {
    .dma_state           = IST8310_IDLE,
    .ist8310_usr_cfg     = my_usr_cfg,
    .ist8310_delay_ms    = my_delay_ms,
    .ist8310_get_stamp_ms = my_get_stamp,
    .ist8310_reset       = my_reset,
    .ist8310_read_reg    = my_read_reg,
    .ist8310_write_reg   = my_write_reg,
    .ist8310_read_regs   = my_read_regs,
    .ist8310_read_regs_dma = my_read_regs_dma,
};

if (IST8310_Init(&ist) == 0) {
    MagneticField mag;
    IST8310_ReadData(&ist, &mag);
    // mag.data.x/y/z → µT
}
```

IST8310 也可以挂在 MPU-6500 的 AUX I2C 总线上，此时平台 I/O 函数直接委托给 `MPU6500_AUXIIC_ReadReg` / `MPU6500_AUXIIC_WriteReg`。

## 接口数据类型

| 类型 | 字段 | 单位 |
|------|------|------|
| `Vector3` | `float x, y, z` | 随上下文 |
| `Imu` | `timestamp_ms`, `acceleration` (Vector3), `angular_velocity` (Vector3) | ms, m/s², deg/s |
| `MagneticField` | `timestamp_ms`, `data` (Vector3) | ms, µT |
| `Temperature_Celsius` | `timestamp_ms`, `data` (float) | ms, °C |

## License

Apache-2.0
