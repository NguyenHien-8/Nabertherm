# Description — Đặc tả thuật toán `main.c` cho AI

## 0. Mục đích tài liệu

Tài liệu này là đặc tả máy đọc được ở mức logic cho chương trình điều khiển lò nung Nabertherm. AI sử dụng tài liệu này phải hiểu, phân tích, sửa đổi hoặc tái tạo thuật toán mà không phá vỡ các bất biến an toàn và thời gian thực.

Tệp nguồn có thẩm quyền: `main.c`.

Các thư viện ngoài được gọi bởi `main.c`:

- `LiquidCrystal_I2C`: giao tiếp LCD.
- `MAX31856`: giao tiếp bộ chuyển đổi can nhiệt.
- `PID_Controller`: PID P-on-error, direct acting, có output sum.
- STM32 HAL: GPIO, EXTI, TIM, I2C, SPI, Flash, tick.

Khi source và tài liệu khác nhau, source hiện tại là nguồn sự thật. Khi sửa source, phải cập nhật lại tài liệu.

---

## 1. Mô hình hệ thống

### 1.1. Plant

- Đối tượng: lò nung điện Nabertherm 3 kW, Tmax danh định 1280 °C.
- Actuator: SSR on/off trên `PB7`.
- Sensor: can nhiệt loại S qua MAX31856.
- Đặc tính: plant chỉ gia nhiệt chủ động, không làm mát chủ động; quán tính và trễ lớn.
- Biến điều khiển: thời gian SSR ON trong cửa sổ 1000 ms.

### 1.2. Hardware contract

```yaml
mcu: STM32F103C8T6
lcd:
  bus: I2C2
  address_7bit: 0x27
  columns: 16
  rows: 2
sensor:
  device: MAX31856
  bus: SPI1
  chip_select: PA15
  thermocouple: type_S
  mains_filter_hz: 50
  conversion_mode: continuous
ssr:
  pin: PB7
  active_level: HIGH
buttons:
  PA8: SETTING_RUN
  PA9: REDIRECT
  PA10: DOWN
  PA11: UP
  PB15: SELECT_OK
timer:
  device: TIM2
  logical_period: 1_second
```

---

## 2. Kiểu và trạng thái

### 2.1. SystemState_t

```yaml
SYS_IDLE:
  timer_increment: false
  heating_allowed: false
SYS_RUNNING:
  timer_increment: true_if_no_fault
  heating_allowed: true_if_sensor_valid_and_no_fault
SYS_COMPLETED:
  timer_increment: false
  heating_allowed: false
```

### 2.2. UIState_t

```yaml
UI_STATE_MAIN: màn hình vận hành
UI_STATE_SET_INTERVAL: chỉnh số lượng khoảng
UI_STATE_SET_P: chọn Mode/Temp/Time của một khoảng
UI_STATE_SET_TEMP: chỉnh 4 chữ số nhiệt độ
UI_STATE_SET_TIME: chỉnh giờ/phút/giây
```

### 2.3. IntervalMode_t

```yaml
MODE_MT:
  semantics: nhiệt độ đích cố định trong toàn khoảng
  end_condition: hết thời gian
MODE_TIOT:
  semantics: tăng nhiệt theo quỹ đạo đến nhiệt độ đích trong thời gian cài
  end_condition: hết thời gian và đạt vùng nhiệt độ cuối
```

### 2.4. MTControlPhase_t

```yaml
MT_PHASE_APPROACH: gia nhiệt có taper, Ki=0
MT_PHASE_COAST: SSR cưỡng bức OFF
MT_PHASE_HOLD: PID giữ nhiệt có Ki
```

### 2.5. ControlFault_t

```yaml
CONTROL_FAULT_NONE: không lỗi
CONTROL_FAULT_MAX31856: thanh ghi fault khác 0
CONTROL_FAULT_SENSOR_DATA: dữ liệu không hợp lệ hoặc MAX31856 chưa sẵn sàng
CONTROL_FAULT_OVERTEMP: raw hoặc filtered >= 1300 °C
```

---

## 3. Dữ liệu chính và đơn vị

```yaml
Input:
  meaning: nhiệt độ lọc đưa vào PID
  unit: degC
raw_temp:
  meaning: nhiệt độ trực tiếp từ MAX31856
  unit: degC
filtered_temp:
  meaning: low-pass filtered temperature
  unit: degC
Setpoint:
  meaning: setpoint điều khiển tức thời
  unit: degC
Output:
  meaning: PID output
  unit: milliseconds_ON_per_1000ms_window
active_output:
  meaning: output sau supervisory limits
  unit: milliseconds_ON_per_1000ms_window
temp_rate_c_per_min:
  meaning: tốc độ biến thiên nhiệt độ đã lọc
  unit: degC_per_minute
Run_Total_Seconds:
  meaning: đồng hồ profile toàn cục
  unit: seconds
Target_Run_Seconds:
  meaning: deadline tuyệt đối của interval hiện tại
  unit: seconds
Current_Interval:
  meaning: số thứ tự interval
  indexing: one_based
```

Quy tắc chỉ số:

```text
interval_index = Current_Interval - 1
```

Không được truy cập `Intervals[Current_Interval]`.

---

## 4. Bất biến bắt buộc

AI sửa hoặc tái tạo code PHẢI duy trì các điều kiện sau:

```text
INV-01: Control_Fault != NONE       => PB7 = LOW.
INV-02: System_Run_State != RUNNING => PB7 = LOW.
INV-03: sensor_has_valid_sample=0   => PB7 = LOW.
INV-04: Input/raw_temp quá ngưỡng   => trip fault, PB7 = LOW.
INV-05: Output và active_output luôn nằm trong 0...1000 ms.
INV-06: ISR nút nhấn chỉ debounce và đặt cờ.
INV-07: ISR TIM2 chỉ tăng đồng hồ và đặt cờ LCD.
INV-08: Không ghi Flash, I2C, SPI blocking dài hoặc chạy PID trong ISR.
INV-09: Vòng lặp chính không được chặn bằng delay.
INV-10: TIOT nonzero-duration không hoàn thành khi chưa đạt target gate.
INV-11: Khi fault được hồi phục, hệ thống không tự restart; giữ SYS_IDLE.
INV-12: Chuyển MANUAL/OFF phải xóa trạng thái công suất có thể gây bật SSR.
INV-13: Flash layout thay đổi => tăng FLASH_DATA_VERSION.
INV-14: Plant không thể làm mát chủ động; target giảm không được gây deadlock TIOT.
```

---

## 5. Thứ tự thực thi

### 5.1. Initialization

```pseudo
HAL_Init()
SystemClock_Config()
MX_GPIO_Init()
MX_TIM2_Init()
MX_I2C2_Init()
MX_SPI1_Init()

Load_Settings_From_Flash()

LCD_init(I2C2, 0x27, 16, 2)
LCD_backlight_on()
LCD_clear()

Start_TIM2_interrupt()

max31856_ready = MAX31856_Init(SPI1, CS=PA15)
IF ready:
    set thermocouple type S
    set noise filter 50 Hz
    set continuous conversion
ELSE:
    Control_Fault = SENSOR_DATA

PID_Init(Input, Output, Setpoint, base gains, P_ON_E, DIRECT)
PID mode = MANUAL
PID output limits = 0..1000
PID sample time = 1000 ms
windowStartTime = HAL_GetTick()
```

### 5.2. Main loop

Thứ tự là bắt buộc:

```pseudo
LOOP forever:
    now_ms = HAL_GetTick()

    Read_Temperature_Task(now_ms)
    Process_Buttons()

    current_run_seconds = atomic/read Run_Total_Seconds
    Advance_Profile_If_Needed(current_run_seconds)
    Update_PID_And_SSR(now_ms, current_run_seconds)
    Update_LCD()
END LOOP
```

Lý do:

- Phải có dữ liệu nhiệt mới trước quyết định an toàn.
- Nút có thể dừng hệ thống trước khi PID phát lệnh.
- Profile phải chuyển interval trước khi tính setpoint.
- LCD hiển thị trạng thái sau cùng của vòng lặp.

---

## 6. Interrupt contract

### 6.1. GPIO EXTI

```pseudo
on GPIO interrupt:
    now = HAL_GetTick()
    IF now - last_time_for_pin > 200 ms:
        set corresponding button flag = 1
        last_time_for_pin = now
```

Không chuyển state trực tiếp trong ISR.

### 6.2. TIM2

```pseudo
on TIM2 period:
    IF System_Run_State == SYS_RUNNING
       AND Control_Fault == NONE:
        Run_Total_Seconds += 1
        IF UI == MAIN:
            LCD_Needs_Update = true
```

---

## 7. Sensor task contract

### Signature

```c
static void Read_Temperature_Task(uint32_t now_ms);
```

### Preconditions

- Được gọi liên tục từ main loop.
- `now_ms` lấy từ `HAL_GetTick()`.

### Scheduling

```pseudo
IF now_ms - last_temp_read_time < 250 ms:
    return
last_temp_read_time = now_ms
```

### Data validation

```pseudo
IF max31856_ready == false:
    Trip_Control_Fault(SENSOR_DATA)
    return

fault_bits = MAX31856_ReadFault()
IF fault_bits != 0:
    reset recovery counters
    Trip_Control_Fault(MAX31856)
    return

measured = MAX31856_ReadThermocoupleTemperature()
valid = finite(measured) AND -50 <= measured <= 1800

IF invalid:
    invalid_temp_count++
    IF count >= 3:
        Trip_Control_Fault(SENSOR_DATA)
    return
```

### Filter

```pseudo
raw_temp = measured
IF first_valid_sample OR filtered_temp_nonfinite:
    filtered_temp = measured
ELSE:
    filtered_temp += 0.20 * (measured - filtered_temp)

Input = filtered_temp
Current_Temp = filtered_temp
sensor_has_valid_sample = true
```

### Rate estimate

Cứ 5000 ms:

```pseudo
instant_rate = (Input - previous_Input) / elapsed_minutes
instant_rate = clamp(instant_rate, -80, +80)
temp_rate += 0.25 * (instant_rate - temp_rate)
```

### Overtemperature

```pseudo
IF measured >= 1300 OR filtered_temp >= 1300:
    Trip_Control_Fault(OVERTEMP)
```

### Fault recovery

- OVERTEMP: filtered <= 1250 trong 4 mẫu hợp lệ liên tiếp.
- MAX31856/SENSOR_DATA: 4 mẫu hợp lệ liên tiếp.
- Khi clear fault: chỉ đặt fault = NONE; không chạy lại profile.

---

## 8. Profile state contract

### 8.1. Start_Profile

```pseudo
sanitize settings

IF fault active OR no valid sensor sample:
    possibly set SENSOR_DATA
    state = IDLE
    stop heating
    return false

atomically:
    run_seconds = 0
    current_interval = 1
    interval_start_sec = 0
    interval_start_temp = Input
    target_run_seconds = duration(P1)
    state = RUNNING

reset PID and supervisory states
initialize MT/TIOT estimator states
skip zero-duration intervals through Advance_Profile_If_Needed(0)
return state == RUNNING
```

### 8.2. Interval duration

```text
duration_sec = hour × 3600 + minute × 60 + second
```

### 8.3. Advance_Profile_If_Needed

```pseudo
WHILE running
  AND current_time >= target_deadline
  AND guard < MAX_INTERVALS:

    finished_index = current_interval - 1
    finished_duration = duration(finished_index)

    IF mode == TIOT
       AND finished_duration > 0
       AND TIOT_Target_Reached(finished_index) == false:
        tiot_deadline_extension = true
        return

    tiot_deadline_extension = false

    IF current_interval < total_intervals:
        boundary_time = max(current_time, old_target_deadline)
        current_interval++
        interval_start_sec = boundary_time
        interval_start_temp = Input if valid else previous target
        target_deadline = boundary_time + duration(new_interval)
    ELSE:
        state = COMPLETED
        Stop_Heating_Control()
```

`guard` ngăn loop vô hạn do nhiều khoảng 0 giây.

---

## 9. Profile setpoint contract

### MODE_MT

```pseudo
profile_setpoint = target_temp
```

### MODE_TIOT

```pseudo
IF duration == 0:
    return target_temp

elapsed = clamp(current_time - start_time, 0, duration)
fraction = elapsed / duration

profile_setpoint =
    start_temp + (target_temp - start_temp) * fraction

return clamp(profile_setpoint, 0, 1280)
```

---

## 10. MODE_MT formal behavior

### 10.1. Prediction

```pseudo
rising_rate = max(temp_rate, 0)
mt_predicted_temp = Input + rising_rate * 4.0
```

### 10.2. State transitions

```yaml
APPROACH_to_COAST:
  any:
    - Input >= target + 0.5
    - raw_temp >= target + 0.5
    - error <= 12
      and rising_rate >= 0.08
      and predicted_temp >= target - 0.3

APPROACH_to_HOLD:
  all:
    - abs(error) <= 1.5
    - abs(temp_rate) <= 0.2

COAST_to_APPROACH:
  all:
    - error > 3.0
    - temp_rate <= 0

COAST_to_HOLD:
  any:
    - error >= 1.0 and temp_rate <= 0.15
    - abs(error) <= 1.5 and abs(temp_rate) <= 0.2

HOLD_to_COAST:
  any:
    - Input >= target + 0.5
    - raw_temp >= target + 0.5
    - temp_rate > 0.2 and predicted_temp >= target + 0.5

HOLD_to_APPROACH:
  condition: error > 3.0
```

### 10.3. Phase entry actions

```yaml
enter_COAST:
  save: active_output as mt_precoast_output if positive
  Output: 0
  active_output: 0
  outputSum: 0

enter_APPROACH:
  Output: 0
  active_output: 0
  outputSum: 0

enter_HOLD:
  initial_bias: 0.25 * mt_precoast_output
  clamp_bias_to: MT_Hold_Output_Cap(target)
  Output: initial_bias
  active_output: initial_bias
  outputSum: initial_bias
  reset_PID_history: true
```

### 10.4. Gains

```yaml
APPROACH:
  Kp: 20
  Ki: 0
  Kd: 250
COAST:
  Kp: 20
  Ki: 0
  Kd: 250
  actuator_forced_off: true
HOLD:
  Kp: 80
  Ki: 0.40
  Kd: 300
```

### 10.5. Output caps

```pseudo
IF phase == COAST OR Input/raw >= target+0.5:
    output = 0

IF phase == APPROACH:
    cap(error):
        >40 -> 1000
        >20 -> 800
        >10 -> 550
        >5  -> 320
        >3  -> 220
        >2  -> 160
        >1  -> 120
        else -> 90

IF phase == HOLD:
    cap = clamp(180 + 0.45*target, 250, 850)
    IF error <= 0: output = 0
    IF error > 0.25 AND temp_rate < 0
       AND 0 < output < 40:
        output = 40
```

Sau khi giới hạn actuator, phải clamp `pid.outputSum` về cùng giới hạn để chống windup.

---

## 11. MODE_TIOT formal behavior

### 11.1. Completion gate

```pseudo
function TIOT_Target_Reached(index):
    IF index invalid OR sensor invalid/nonfinite:
        return false

    target = Intervals[index].Temp

    IF target < Current_Interval_Start_Temp - 1:
        return true  # không deadlock với target giảm

    return abs(Input-target) <= 1
       AND abs(raw_temp-target) <= 2
```

### 11.2. Deadline-aware command setpoint

Inputs:

- `profile_setpoint`: quỹ đạo tuyến tính.
- `Input`: nhiệt đo.
- `target`: nhiệt cuối.
- `remaining_sec`: thời gian còn lại.
- `temp_rate`: tốc độ đo.

```pseudo
planned_rate =
    (target - start_temp) * 60 / duration_sec

IF remaining_sec > 0 AND target > Input:
    required_rate = (target - Input) / remaining_minutes
ELSE IF target - Input > 1:
    required_rate = 40
ELSE:
    required_rate = 0

required_rate = clamp(required_rate, 0, 40)

pace_error = profile_setpoint - Input
positive_rate = max(temp_rate, 0)
rate_deficit = required_rate - positive_rate

lead = planned_rate * 1.0
lead += max(pace_error, 0) * 1.20
lead += max(rate_deficit, 0) * 1.20
lead = clamp(lead, 0, 45)

command_setpoint = profile_setpoint + lead

IF deadline expired:
    command_setpoint = target

command_setpoint = clamp(command_setpoint, 0, target)
command_setpoint = clamp(command_setpoint, 0, 1280)
```

Với target không tăng hoặc duration 0, command setpoint là target.

### 11.3. Gains

```yaml
TIOT:
  Kp: 35
  Ki:
    value: 0.10
    enabled_when: abs(command_setpoint - Input) <= 30
  Kd: 250
```

Khi ngoài vùng Ki, phải đặt `outputSum = 0`.

### 11.4. Adaptive full-power rate

Initial value:

```text
5.0 °C/min
```

Update only if:

```text
system RUNNING
last_control_mode == MODE_TIOT
active_output >= 250 ms
temp_rate >= 0.10 °C/min
interval_elapsed >= 60 s
target - Input > 10 °C
```

Estimate:

```pseudo
observed_full_rate = temp_rate * 1000 / active_output
observed_full_rate = clamp(observed_full_rate, 0.5, 40)
estimate += 0.08 * (observed_full_rate - estimate)
```

### 11.5. Output floor

```pseudo
rate_ff =
    required_rate / max(full_rate_est, 0.5)
    * 1000
    * 1.05

corrective =
    max(pace_error, 0) * 20
    + max(rate_deficit, 0) * 35

output_floor = max(rate_ff, corrective)
output_floor = clamp(output_floor, 0, 1000)

IF deadline expired AND target_error > 1:
    output_floor = 1000

limited_output = max(PID_output, output_floor)
```

### 11.6. Final predictive coast and cap

```pseudo
predicted_temp = Input + max(temp_rate, 0) * 2.5

IF target_error <= 10
   AND positive_rate >= 0.08
   AND predicted_temp >= target - 0.25:
    limited_output = 0
    Output = 0
    outputSum = 0
ELSE:
    cap = 1000
    IF target_error <= 1.5: cap = 180
    ELSE IF target_error <= 4.0: cap = 350
    clamp limited_output and outputSum to cap

IF target_error <= 0:
    limited_output = 0
    Output = 0
    outputSum = 0

IF Input/raw >= target + 0.5:
    force output = 0 and clear integral

IF 20 < limited_output < 40
   AND (pace_error > 0.5 OR rate_deficit > 0.2):
    limited_output = 40
```

---

## 12. PID and SSR execution contract

### PID

```yaml
controller_direction: DIRECT
proportional_mode: P_ON_E
sample_time_ms: 1000
output_min: 0
output_max: 1000
```

`Apply_PID_Tunings()` chỉ gọi `PID_SetTunings()` nếu gain hiển thị thay đổi. Điều này tránh ghi lại scaling nội bộ không cần thiết.

Khi Setpoint giảm hơn 0.25 °C so với vòng trước:

```text
pid.outputSum = 0
```

### SSR

```pseudo
ms_in_window = (now_ms - windowStartTime) mod 1000

IF active_output <= 20:
    PB7 = LOW
ELSE IF active_output >= 980:
    PB7 = HIGH
ELSE:
    PB7 = HIGH if ms_in_window < floor(active_output)
          else LOW
```

Không dùng PWM tần số cao.

---

## 13. UI event contract

Sự kiện được snapshot và xóa nguyên tử bằng cách tạm tắt IRQ.

Priority:

1. `SETTING/RUN`.
2. `REDIRECT`.
3. `SELECT`.
4. `UP/DOWN`.

### SETTING/RUN

```pseudo
IF UI == MAIN:
    UI = SET_INTERVAL
    system = IDLE
    Stop_Heating_Control()
ELSE:
    Commit_Pending_Edit()
    Sanitize_Settings()
    Save_Settings_To_Flash()
    UI = MAIN
    Start_Profile()
return
```

### REDIRECT

```yaml
SET_P: next interval, wrap 1..Total
SET_TEMP: next digit, wrap 0..3
SET_TIME: next field, wrap 0..2
```

### SELECT

```yaml
SET_INTERVAL: enter SET_P, P=1, cursor=Mode
SET_P+Mode: toggle MT/TIOT
SET_P+Temp: copy value to edit buffer, enter SET_TEMP
SET_P+Time: copy values to edit buffer, enter SET_TIME
SET_TEMP: commit and return SET_P
SET_TIME: commit and return SET_P
```

### UP/DOWN

Chỉ xử lý nếu đúng một trong hai cờ có giá trị. Nếu cả hai cùng có thì bỏ qua.

---

## 14. LCD contract

- Chỉ render khi `LCD_Needs_Update == true`.
- Mỗi hàng luôn được pad đủ 16 ký tự.
- Chỉ ghi hàng có nội dung khác cache trước.
- Blink cursor:
  - SET_TEMP: cột `7 + Temp_Digit_Index`.
  - SET_TIME: cột 6, 9 hoặc 12.
- Fault display có ưu tiên hơn thông tin vận hành.

---

## 15. Flash persistence contract

```yaml
address: 0x0800FC00
magic: 0xAABBCCDE
version: 2
checksum: FNV-1a_32
checksum_range: từ đầu Flash_Data_t đến ngay trước trường Checksum
```

Write sequence:

```pseudo
Sanitize_Settings()
build zero-initialized Flash_Data_t
populate header and payload
calculate checksum
disable IRQ
unlock flash
erase one page
program 32-bit words
lock flash
restore IRQ
verify magic, version and checksum in place
```

Load sequence:

```pseudo
read pointer at address
verify magic, version, TotalIntervals range and checksum
IF valid:
    copy payload to RAM
    sanitize
ELSE:
    initialize one default MT interval
```

Không được ghi Flash từ ISR.

---

## 16. Function side-effect map

| Function | Reads | Writes/side effects |
|---|---|---|
| `Read_Temperature_Task` | MAX31856, tick state | sensor vars, rate, fault, LCD flag |
| `Process_Buttons` | event flags, UI state | UI, intervals, Flash, run state |
| `Advance_Profile_If_Needed` | clock, intervals, sensor | interval indices, deadlines, completion |
| `Update_PID_And_SSR` | state, sensor, profile | PID, supervisory vars, PB7 |
| `Update_LCD` | all display state | I2C LCD and caches |
| `Stop_Heating_Control` | PID mode | PB7 LOW, PID MANUAL, resets |
| `Trip_Control_Fault` | requested fault | fault latch, IDLE, SSR OFF |
| `Save_Settings_To_Flash` | RAM settings | internal Flash |
| `HAL_GPIO_EXTI_Callback` | GPIO pin, tick | button event flags |
| `HAL_TIM_PeriodElapsedCallback` | run/fault state | run clock, LCD flag |

---

## 17. Quy tắc dành cho AI khi chỉnh sửa

### Phải làm

- Đọc cả `main.c`, `PID_Controller.*`, `MAX31856.*` trước khi thay đổi công thức liên quan.
- Giữ API HAL và pin map nếu người dùng không yêu cầu đổi phần cứng.
- Giữ luồng không chặn.
- Giữ tất cả điều kiện fail-safe.
- Tách rõ:
  - profile generation;
  - feedback PID;
  - feed-forward/supervisory limit;
  - actuator drive.
- Reset tích phân khi thay đổi mode, giảm Setpoint, cutoff hoặc fault.
- Khi thêm biến persistent, cập nhật Flash version và migration/default.
- Khi chỉnh gain, ghi rõ đơn vị và SampleTime 1 giây.
- Khi chỉnh TIOT, kiểm tra cả deadline tracking và overshoot.
- Khi chỉnh MT, kiểm tra cả ba pha APPROACH/COAST/HOLD.

### Không được làm

- Không bảo đảm bằng lời rằng ±1 °C sẽ đạt trên mọi tải nếu chưa kiểm thử lò thật.
- Không xóa deadline extension để “đúng thời gian” bằng cách chuyển P khi nhiệt chưa đạt.
- Không cho output floor TIOT vô hiệu hóa cutoff/quá nhiệt.
- Không bật SSR trong `SYS_IDLE`, `SYS_COMPLETED` hoặc fault.
- Không đặt code ghi LCD/Flash trong callback ngắt.
- Không thay `Output` sang phần trăm mà không sửa toàn bộ mapping.
- Không dùng `Current_Interval` như chỉ số zero-based.
- Không khởi động lại profile tự động sau khi lỗi hồi phục.

---

## 18. Tiêu chí nghiệm thu sau khi sửa

### Static checks

```text
- Build không warning/error mới.
- Không truy cập ngoài Intervals[0..8].
- Output luôn clamp 0..1000.
- Mọi đường fault đều dẫn tới PB7 LOW.
- Không có HAL_Delay trong while(1).
- Không có Flash/LCD/PID trong ISR.
```

### Functional checks

```text
- PA8 từ MAIN luôn tắt lò và vào setup.
- PA8 từ setup luôn save rồi chỉ start nếu sensor hợp lệ.
- MT chuyển đủ APPROACH -> COAST -> HOLD.
- MT cutoff khi raw hoặc filtered vượt target + 0.5.
- TIOT không chuyển interval khi timeout nhưng chưa đạt gate.
- TIOT chuyển interval khi filtered ±1 và raw ±2.
- Fault MAX31856, invalid data và overtemp đều tắt SSR.
- Sau clear fault, hệ thống giữ IDLE.
- LCD không nhấp nháy do clear toàn màn hình liên tục.
```

### Real-furnace validation

Cần ghi log tối thiểu:

```yaml
timestamp_s
interval
mode
target_temp
profile_setpoint
command_setpoint
raw_temp
filtered_temp
temp_rate_c_per_min
pid_output_ms
active_output_ms
MT_phase
TIOT_required_rate
TIOT_full_power_rate_est
fault
```

Đánh giá:

- overshoot;
- undershoot tại deadline;
- settling time;
- sai số giữ nhiệt;
- duty SSR;
- ảnh hưởng của tải và nhiệt độ làm việc.

---

## 19. Pseudocode tổng hợp

```pseudo
INITIALIZE hardware, flash, sensor, PID
PID = MANUAL
SSR = OFF

FOREVER:
    acquire and validate temperature
    update filtered temperature and heating rate
    process button event flags

    IF interval deadline reached:
        IF TIOT nonzero AND target gate not reached:
            hold current interval at final target
        ELSE:
            move to next interval or complete

    IF not RUNNING OR fault OR sensor invalid:
        stop heating
    ELSE:
        identify current mode
        reset controller state on interval/mode transition

        profile_sp = calculate profile setpoint

        IF mode == MT:
            Setpoint = target
            update APPROACH/COAST/HOLD
            select phase gains
            PID compute
            apply MT cutoff and output cap
        ELSE:
            Setpoint = deadline-aware TIOT command setpoint
            select TIOT gains and integral zone
            PID compute
            apply adaptive rate feed-forward
            apply final predictive coast and caps

        clamp output 0..1000
        drive SSR using 1000 ms time window

    render LCD only when needed
END FOREVER
```
