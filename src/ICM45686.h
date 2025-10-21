/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#ifndef ICM456xx_H
#define ICM456xx_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

extern "C" {
#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"
#include "invn/InvError.h"
#if defined(ICM45686S) || defined(ICM45605S)
#include "imu/inv_imu_edmp_gaf.h"
#endif
}

// algo 0, enable GRV when enable 6-axis(AG)
// algo 1, enable GMRV when enable 6-axis(AM)
// algo 2, enable RV when enable 9-axis(AGM)
enum {
  ALGO_GRV = 0,
  ALGO_GMRV,
  ALGO_RV
};

enum {
  ICM456XX_APEX_TILT=0,       // ​​倾斜
  ICM456XX_APEX_PEDOMETER,    // 计步器​​
  ICM456XX_APEX_TAP,          // 敲击
  ICM456XX_APEX_R2W,          // 抬手亮屏
  ICM456XX_APEX_FF,           // 自由落体
  ICM456XX_APEX_LOWG,         // ​​低重力冲击
  ICM456XX_APEX_HIGHG,        // 高重力冲击
  ICM456XX_APEX_MAX
};

// This defines the handler called when retrieving a sample from the FIFO
//typedef void (*ICM456xx_sensor_event_cb)(inv_imu_sensor_data_t *event);
// This defines the handler called when receiving an irq
typedef void (*ICM456xx_irq_handler)(void);

class ICM456xx {
  public:
    ICM456xx(TwoWire &i2c,bool address_lsb, uint32_t freq);
    ICM456xx(TwoWire &i2c,bool address_lsb);
    ICM456xx(SPIClass &spi,uint8_t chip_select_id, uint32_t freq);
    ICM456xx(SPIClass &spi,uint8_t chip_select_id);
    int begin();

    // 启动加速度计​​。
    // odr: 输出数据速率(ODR)​，单位HZ
    // fsr: 满量程范围(FSR)​，数值越大越能测更剧烈的加速度，但灵敏度降低；小量程适合测量微小的运动或稳定的姿态，精度和灵敏度更高​​
    int startAccel(uint16_t odr, uint16_t fsr);

    // 启动陀螺仪。
    // odr: 输出数据速率(ODR)​，单位HZ
    // fsr: 满量程范围(FSR)​，决定陀螺仪能测量的最大角速度。范围越大，能测量的转速上限越高，但相同范围内的测量精度会降低。
    int startGyro(uint16_t odr, uint16_t fsr);
    
    // 从IMU传感器的寄存器中同步读取原始数据​​
    // 从ACCEL_DATA_X1_UI地址开始，读取sizeof(inv_imu_sensor_data_t)共14个字节的数据，其中包含加速度计、陀螺仪、温度数据
    int getDataFromRegisters(inv_imu_sensor_data_t& data);

    // 配置和启用IMU传感器的​​FIFO（先进先出）中断功能​​
    int enableFifoInterrupt(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t fifo_watermark);

    // 从IMU传感器的FIFO（先进先出）中读取数据
    int getDataFromFifo(inv_imu_fifo_data_t& data);

#if defined(ICM45686S) || defined(ICM45605S) || defined(ICM45608) || defined(ICM45689)
    int startGaf(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t algo);
    int getGafData(inv_imu_edmp_gaf_outputs_t& gaf_outputs);
    int getGaf_GRVData(float& quatW,float& quatX,float& quatY,float& quatZ);
    int getGaf_GMRVData(float& quatW,float& quatX,float& quatY,float& quatZ);    
    int getGaf_RVData(float& quatW,float& quatX,float& quatY,float& quatZ);
    int getGaf_RMData(float& mX,float& mY, float& mZ);
#endif
    int stopAccel(void);
    int stopGyro(void);

    // 启动基于​​倾斜检测功能。一旦发现倾斜，就通过指定的引脚发出信号，并调用指定的函数来处理。
    int startTiltDetection(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);

    // 启动计步器功能
    int startPedometer(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);

    // 启动​​自由落体检测功能
    int startFreeFall(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);

    // 启动高重力冲击​​检测功能​​（瞬时、剧烈的加速度变化，如：设备跌落检测、碰撞感知、冲击记录）
    int startHighG(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);

    // 启动低重力重启检测功能（失重状态，持续、平缓的加速度变化，如：自由落体检测、倾斜感知、运输振动监测）
    int startLowG(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);

    // 启动运动唤醒功能（极低功耗下感知微小运动以唤醒系统）
    int startWakeOnMotion(uint8_t intpin, ICM456xx_irq_handler handler);
    
    // 启动设备被​​敲击​​检测功能
    int startTap(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    
    // 启动抬手动作检测功能
    int startRaiseToWake(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);    

    // 获取计步数据：总步数、步频和活动类型
    int getPedometer(uint32_t& step_count, float& step_cadence, char*& activity);

    // 检查自上次调用以来是否发生过倾斜事件
    int getTilt(void);

    // 获取敲击事件的详细信息：次数、方向和轴
    int getTap(uint8_t& tap_count, uint8_t& axis, uint8_t& direction);

    // 检查是否检测到“抬手亮屏”的抬手动作
    int getRaiseToWake(void);

    // 获取自由落体事件的持续时间
    int getFreefall(uint32_t& duration_ms);

    // 检查是否发生过高重力冲击事件
    int getHighG(void);

    // 检查是否发生过低重力或失重事件
    int getLowG(void);

    // ​​同步APEX引擎状态​​
    int updateApex(void);

    // 配置APEX中断
    int setApexInterrupt(uint8_t intpin, ICM456xx_irq_handler handler);

    inv_imu_edmp_int_state_t apex_status;

  protected:
    inv_imu_device_t icm_driver;
    accel_config0_accel_odr_t accel_freq_to_param(uint16_t accel_freq_hz);
    gyro_config0_gyro_odr_t gyro_freq_to_param(uint16_t gyro_freq_hz);
    accel_config0_accel_ui_fs_sel_t accel_fsr_g_to_param(uint16_t accel_fsr_g);
    gyro_config0_gyro_ui_fs_sel_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps);
    int setup_irq(uint8_t intpin, ICM456xx_irq_handler handler);
    int startAPEX(dmp_ext_sen_odr_cfg_apex_odr_t edmp_odr, accel_config0_accel_odr_t accel_odr);
    uint32_t step_cnt_ovflw;
    bool apex_enable[ICM456XX_APEX_MAX];
    dmp_ext_sen_odr_cfg_apex_odr_t apex_edmp_odr;
    accel_config0_accel_odr_t apex_accel_odr;
};

#endif // ICM456xx_H
