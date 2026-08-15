FURNACE SAFETY UPDATE - MAX31856 + IWDG
==========================================

Mục tiêu
--------
- Loại bỏ nguy cơ MAX31856/SPI giữ MCU trong HAL_MAX_DELAY.
- Thêm Independent Watchdog (IWDG) để tự reset nếu foreground loop bị treo.
- GIỮ NGUYÊN thuật toán MODE_MT, MODE_TIOT và PID.

1. MAX31856
-----------
- Mỗi giao dịch SPI có timeout 10 ms.
- Tối đa 2 lần thử cho một transaction hoàn chỉnh.
- CS luôn được kéo HIGH sau mọi đường thoát, kể cả HAL_TIMEOUT/HAL_ERROR.
- Driver ghi nhớ:
    communication_error
    last_hal_status
    communication_error_count
- ReadFault() trả 0xFF khi SPI lỗi để legacy code cũng fail-safe.
- ReadThermocoupleTemperature() trả NAN khi SPI lỗi.
- main.c kiểm tra communication_error ngay sau ReadFault/ReadTemperature và
  gọi Trip_Control_Fault(CONTROL_FAULT_MAX31856) => SSR OFF.

2. IWDG
-------
- Lập trình trực tiếp thanh ghi STM32F1, KHÔNG phụ thuộc HAL_IWDG module /
  stm32f1xx_hal_iwdg.c / CubeMX enable.
- Prescaler: /64.
- Reload: 2499.
- Với LSI danh định 40 kHz: timeout xấp xỉ 4 giây.
- IWDG chỉ được refresh ở cuối một vòng while(1) hoàn chỉnh.
- Nếu SPI hoặc code bị treo vô hạn => không refresh => hardware reset.
- Error_Handler() kéo PB7 LOW trước, sau đó không refresh watchdog.

3. Những phần KHÔNG thay đổi
-----------------------------
Đã so sánh byte-for-byte phần thân các hàm:
- Calculate_Profile_Setpoint
- Calculate_TIOT_Control_Setpoint
- Update_TIOT_Rate_Estimate
- Reset_TIOT_Fast_Rate
- Update_TIOT_Fast_Rate
- Limit_TIOT_Output
- Reset_MT_Control
- Update_Temperature_Rate
- Apply_PID_Tunings
- MT_Hold_Output_Cap
- MT_Hold_Initial_Bias
- Enter_MT_Phase
- Update_MT_Control_Phase
- Limit_MT_Output
- Update_PID_And_SSR

Tất cả #define MT_* và TIOT_* cũng giữ nguyên.

4. File cần thay trong project
-------------------------------
Core/Src/main.c
Core/Src/MAX31856.c
Core/Inc/MAX31856.h

main.h và LiquidCrystal_I2C.* trong package là bản gốc, chỉ kèm để dễ đối chiếu.

5. Kiểm tra đã thực hiện
-------------------------
- MAX31856.c compile syntax với HAL stub: PASS.
- Test giả lập SPI timeout:
    ReadFault() -> 0xFF
    communication_error -> true
    không block vô hạn
  Kết quả: PASS.
- Kiểm tra MT/TIOT function hashes: identical.

6. Khuyến nghị test thực tế
----------------------------
A. Chạy bình thường MT/TIOT để xác nhận đặc tính nhiệt không đổi.
B. Khi đang RUN, tháo đường SPI hoặc ngắt nguồn MAX31856:
   - SSR phải OFF nhanh.
   - máy chuyển fault/IDLE.
C. Test watchdog:
   - tạm thời thêm while(1) ở một vị trí foreground SAU khi IWDG đã init;
   - MCU phải reset sau vài giây.
   - xóa đoạn while(1) sau test.
D. Luôn nên có điện trở pulldown phần cứng trên chân điều khiển SSR và bộ
   bảo vệ quá nhiệt độc lập nếu dùng vận hành không giám sát.
