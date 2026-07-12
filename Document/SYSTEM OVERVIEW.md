# SYSTEM OVERVIEW & FIRMWARE SPECIFICATION
**Project:** Nabertherm Furnace Control Panel Firmware
**MCU:** STM32F103C8T6 (Blue Pill / Custom Board)
**Framework/Language:** C / STM32CubeIDE / HAL Library

Bạn hãy đóng vai là một kỹ sư lập trình nhúng (Embedded Software Engineer). Dựa vào các thông số, cấu trúc hệ thống và thư viện được mô tả dưới đây, hãy viết toàn bộ source code cho file `main.c` (bao gồm khai báo biến, state machine, xử lý ngắt/nút nhấn và vòng lặp while). Đảm bảo logic xử lý thời gian và chuyển tiến trình hoạt động chính xác theo mô tả.

---

## 1. Functional Requirements Overview
Hệ thống cần quản lý lò nung Nabertherm với các tính năng:
1. Đọc và hiển thị nhiệt độ thực tế từ cảm biến.
2. Quản lý thời gian thực (Real-time: hh/mm/ss) của từng chu kỳ.
3. Quản lý các khoảng thời gian nung (Intervals/Segments progress).
4. Cho phép người dùng tương tác qua 5 nút nhấn để cài đặt các thông số.
5. Hiển thị thông tin lên màn hình LCD I2C 16x2.

---

## 2. Hardware Configuration & Pin Mapping
- **MCU:** STM32F103C8T6.
- **Clock Source:** HSE (High-Speed External) 8MHz, PLL up to 72MHz.
- **Timer (RTC Software):** TIM2, tạo ngắt 1Hz (1 giây) để đếm thời gian.
- **Cảm biến nhiệt độ:** MAX31856 (SPI1: `PB3`-SCK, `PB4`-MISO, `PB5`-MOSI, `PA15`-CSS). *Sử dụng thư viện `MAX31856.h`*.
- **Màn hình LCD:** LCD 16x2 (I2C2: `PB10`-SCL, `PB11`-SDA), địa chỉ mặc định `0x27`. *Sử dụng thư viện `LiquidCrystal_I2C.h`*.
* **Nút nhấn (EXTI, Software Debounce = 200ms bằng HAL_GetTick):**
    * `PA8` (Setting/Run): Chuyển đổi giữa chế độ chạy và chế độ cài đặt.
    * `PA9` (Redirect): Chuyển đổi qua lại giữa các Menu P1, P2... hoặc di chuyển con trỏ chỉnh sửa thông số.
    * `PB15` (OK/Select): Xác nhận, lưu, hoặc chọn mục đang được trỏ tới.
    * `PA11` (UP): Tăng giá trị hoặc di chuyển con trỏ lên.
    * `PA10` (DOWN): Giảm giá trị hoặc di chuyển con trỏ xuống.
* **Relay SSR:** GPIO `PB7` (Điều khiển gia nhiệt - Cải tiến sau này).

---

## 3. Data Structure & Global Variables (Cấu trúc dữ liệu & Biến trạng thái)
Cần thiết kế mảng Struct và các biến toàn cục để quản lý logic:
* **Mảng `Interval_TypeDef`:** Chứa cài đặt của từng Interval (Mode, Temp, Time_Hour, Time_Min, Time_Sec).
* **Biến `System_Run_State`:** Có 3 trạng thái: 
    * `IDLE`: Vừa cấp điện, chưa chạy.
    * `RUNNING`: Đang chạy tiến trình.
    * `COMPLETED`: Đã chạy xong tất cả các tiến trình.
* **Biến `Current_Interval`:** Lưu tiến trình P đang chạy (Từ 1 đến X).
* **Biến đếm thời gian thực:** `Run_Hour`, `Run_Min`, `Run_Sec`.

---

## 4. Functional Requirements Detail & State Machine

### 4.1. STATE_MAIN (Màn hình chính & Quản lý vận hành)
Màn hình chính phải hoạt động dựa trên biến `System_Run_State`:
* **Dòng 1:** Hiển thị nhiệt độ thực tế và tiến trình (VD: `Temp: 1280 °C  P1/3`). Nếu ở IDLE thì hiện `P0/X`.
* **Dòng 2:** Hiển thị thời gian chạy (VD: `Time: 00:00:00`).
* **Logic trạng thái IDLE (Vừa cấp điện):** Thời gian đứng yên ở `00:00:00`. Timer ngắt nhưng KHÔNG tăng biến đếm thời gian.
* **Logic trạng thái RUNNING:** * Timer 1Hz tăng biến đếm thời gian tổng (`Run_Hour`, `Run_Min`, `Run_Sec`) liên tục. **TUYỆT ĐỐI KHÔNG RESET** các biến thời gian này về 0 khi chuyển tiến trình P.
    * **Cơ chế Auto-step (Cộng dồn thời gian):** * Hệ thống cần sử dụng một biến cờ mốc thời gian đích (VD: `Target_Run_Seconds`). 
        * Khi bắt đầu chạy P1, `Target_Run_Seconds = Thời gian cài đặt của P1` (tính ra giây).
        * Trong vòng lặp chính, liên tục quy đổi thời gian đang chạy hiện tại ra giây và so sánh với `Target_Run_Seconds`.
        * **Khi thời gian hiện tại ĐẠT HOẶC VƯỢT `Target_Run_Seconds`:** Tự động tăng `Current_Interval` sang P tiếp theo (P+1). Đồng thời, cập nhật lại mốc thời gian đích mới: `Target_Run_Seconds = Target_Run_Seconds + Thời gian cài đặt của P+1` (tính ra giây). 
        * Cập nhật thông số tiến trình P mới lên LCD ngay lập tức, trong khi thời gian trên LCD vẫn đếm tiến lên liên tục không đứt quãng.
* **Logic trạng thái COMPLETED:** Khi đã chạy xong tiến trình P cuối cùng, đổi state sang COMPLETED. Dừng đếm thời gian. Giữ nguyên màn hình hiển thị nhiệt độ hiện tại và thời gian kết thúc của P cuối cùng (Không cần màn hình Finish mới, giữ nguyên trạng thái cũ).
* **Nút nhấn PA8:** Khi đang ở STATE_MAIN (bất kể IDLE, RUNNING hay COMPLETED), nhấn `PA8` sẽ chuyển sang hệ thống cài đặt (Setting Mode). Lúc này hệ thống tự động ngắt cờ RUNNING (chuyển về IDLE để tạm dừng nung).

### 4.2. SETTING MODE: STATE_SET_INTERVAL (Cài đặt tổng số Interval)
* **Mô tả:** Trạng thái đầu tiên khi vào Setting Mode. Dùng để cấu hình tổng số chu kỳ P.
* **Dòng 1:** `[SET INTERVAL]`
* **Dòng 2:** `Quantity: X` (X là số lượng từ 1 đến N).
* **PA11 (UP) / PA10 (DOWN):** Tăng/giảm số lượng X.
* **PB15 (OK):** Lưu tổng số lượng X và chuyển sang `STATE_SET_P`.
* **Logic PA8:** Khi nhấn PA8 để lưu và thoát khỏi bất kỳ giao diện Setting nào (SET_INTERVAL, SET_P, SET_TEMP, SET_TIME) để về lại `STATE_MAIN`:
    * Set `Run_Hour = 0`, `Run_Min = 0`, `Run_Sec = 0`.
    * Set `Current_Interval = 1`.
    * Set `System_Run_State = RUNNING` (Bắt đầu chạy lại từ P1 với thời gian 00:00:00).

### 4.3. SETTING MODE: STATE_SET_P (Menu cấu hình từng Interval)
* **Mô tả:** Màn hình 16x2 chỉ hiện 2 dòng, nhưng Menu có 3 mục (Mode, Set Temp, Set Time). Sử dụng biến quản lý cuộn trang (Menu Index) để dịch chuyển dòng hiển thị khi nhấn UP/DOWN.
* **Dòng 1:** `==== SET Px ====` Luôn là Header ở trên cùng.
* **Dòng 2:** Các lựa chọn `>Mode: MT`, `>Set Temp`, `>Set Time`.
* **PA11 / PA10:** Di chuyển con trỏ `>` lên/xuống và cuộn trang hiển thị.
* **PA9:** Chuyển đổi nhanh giữa các P (VD: Đang cài P1, nhấn PA9 sang cài P2).
* **PB15 (Select):** Hoạt động dựa theo vị trí con trỏ `>`:
    * Nếu trỏ ở `Mode`: Nhấn PB15 để đảo trạng thái giữa `MT` và `TIOT`.
    * Nếu trỏ ở `Set Temp`: Nhấn PB15 để đi vào `STATE_SET_TEMP`.
    * Nếu trỏ ở `Set Time`: Nhấn PB15 để đi vào `STATE_SET_TIME`.
* **PA8:** Lưu toàn bộ và thoát về `STATE_MAIN`và gọi logic Reset khởi động lại như mục 4.2.

### 4.4. SETTING MODE: STATE_SET_TEMP (Cài đặt nhiệt độ cho P đang chọn)
* **Dòng 1:** `[SET TEMP Px]`
* **Dòng 2:** `Thres: XXXX °C` (Chữ số đang được chỉnh sửa sẽ nhấp nháy).
* **PA9:** Dịch chuyển vị trí nhấp nháy theo vòng lặp: Nghìn -> Trăm -> Chục -> Đơn vị -> Nghìn.
* **PA11 / PA10:** Tăng/giảm giá trị của chữ số.
* **PB15:** Lưu nhiệt độ, thoát ra lại `STATE_SET_P`.
* **PA8:** Lưu toàn bộ và thoát về `STATE_MAIN` và gọi logic Reset khởi động lại như mục 4.2.

### 4.5. SETTING MODE: STATE_SET_TIME (Cài đặt thời gian cho P đang chọn)
* **Dòng 1:** `[SET TIME Px]`
* **Dòng 2:** `Time: hh:mm:ss` (Cụm thời gian đang chỉnh sửa sẽ nhấp nháy).
* **PA9:** Dịch chuyển vị trí nhấp nháy theo vòng: Giờ (hh) -> Phút (mm) -> Giây (ss) -> Giờ.
* **PA11 / PA10:** Tăng/giảm giá trị thời gian (Lưu ý giới hạn phút/giây <= 59).
* **PB15:** Lưu thời gian, thoát ra lại `STATE_SET_P`.
* **PA8:** Lưu toàn bộ và thoát về `STATE_MAIN` và gọi logic Reset khởi động lại như mục 4.2.

---

## 5. Coding Constraints (Yêu cầu kỹ thuật cốt lõi)
- **Non-blocking Delay:** KHÔNG sử dụng `HAL_Delay()` trong main loop. Update LCD và đọc cảm biến qua `HAL_GetTick()`.
- **Hàm cập nhật hiển thị LCD:** Viết hàm Update_LCD() dựa trên Flag. Chỉ gọi hàm xóa (lcd_clear) và ghi đè dòng dữ liệu khi Trạng thái (State) thay đổi hoặc Giá trị (Value) thay đổi để tránh màn hình bị nháy/giật.
- **Cấu hình Timer & Đếm thời gian:** Trong ngắt `HAL_TIM_PeriodElapsedCallback`để tăng biến đếm thời gian thực hour, minute, second. Chỉ tăng biến thời gian khi `System_Run_State == RUNNING`. Nếu IDLE hoặc COMPLETED thì tuyệt đối không tăng.
- **Nút nhấn (Debounce):** `HAL_GPIO_EXTI_Callback` dùng `HAL_GetTick()` kiểm tra thời gian 200ms. KHÔNG dùng `while` chặn ngắt.

---

## 6. Future Implementations (Cải tiến sau này)
- Tích hợp PID từ `PID_Controller.h` để điều khiển Relay chân `PB7`.
- MT (Maintain Temp): Dùng PID giữ nhiệt độ cố định.
- TIOT (Temperature Increases Over Time): Tính toán hệ số góc Ramp Rate để tăng dần nhiệt độ Setpoint của PID theo hàm thời gian đã cài đặt.