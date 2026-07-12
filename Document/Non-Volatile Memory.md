## 7. Non-Volatile Memory (Lưu trữ cài đặt vào Flash MCU)

**Yêu cầu:** Bổ sung tính năng lưu trữ các thông số cài đặt (Total Intervals, Mode, Temp, Time của từng P) vào bộ nhớ Internal Flash của STM32F103C8T6 để không bị mất dữ liệu khi ngắt nguồn điện (Power loss recovery).

Dựa vào file `main.c` hiện tại, hãy viết thêm các hàm xử lý Flash và chèn vào đúng các vị trí theo logic dưới đây:

### 7.1. Cấu trúc dữ liệu và Địa chỉ Flash (Flash Architecture)
* **Vị trí lưu trữ:** Sử dụng **Page 63** của STM32F103C8T6 (Địa chỉ bắt đầu: `0x0800FC00`). Đây là page cuối cùng của Flash 64KB, đảm bảo không bị ghi đè lên vùng chứa Application Code.
* **Biến cần lưu:** Biến `Total_Intervals` và toàn bộ mảng `Intervals`.
* **Magic Word Validation:** Để MCU phân biệt được Flash đang chứa dữ liệu hợp lệ hay là MCU mới tinh (chứa toàn `0xFF`), hãy định nghĩa một **Magic Word** (ví dụ: `0xAABBCCDD`). 
* **Tạo Struct gom dữ liệu:** Hãy tạo một struct `Flash_Data_t` chứa: Magic Word, `Total_Intervals`, và mảng `Intervals`. Viết hàm chuyển đổi struct này thành mảng uint32_t (Words) để ghi xuống Flash.

### 7.2. Các hàm chức năng cần viết (HAL Flash API)
Viết 2 hàm giao tiếp Flash:
1. `void Save_Settings_To_Flash(void)`:
   * **Bắt buộc:** Phải gọi `HAL_FLASH_Unlock()`.
   * **Bắt buộc:** Phải thực hiện Erase Page 63 (`FLASH_PageErase`) trước khi ghi. Cấu hình `FLASH_EraseInitTypeDef`.
   * Tiến hành ghi (`HAL_FLASH_Program`) từng Word (32-bit) của dữ liệu vào Flash.
   * **Bắt buộc:** Gọi `HAL_FLASH_Lock()` sau khi xong.
   * **An toàn:** Trước khi thao tác xóa/ghi Flash, phải vô hiệu hóa ngắt toàn cục bằng `__disable_irq()` và bật lại `__enable_irq()` sau khi ghi xong để tránh bị ngắt Timer/Nút nhấn làm gián đoạn gây HardFault.

2. `void Load_Settings_From_Flash(void)`:
   * Đọc Word đầu tiên tại địa chỉ `0x0800FC00`. 
   * Nếu khớp với **Magic Word**: Tiến hành đọc các Words tiếp theo và map ngược lại vào biến `Total_Intervals` và mảng `Intervals`.
   * Nếu KHÔNG khớp (Flash trống hoặc lỗi): Gọi lại hàm `Init_Default_Intervals()` để nạp giá trị mặc định.

### 7.3. Vị trí tích hợp vào State Machine (Integration Points)
Tuyệt đối KHÔNG ghi vào flash liên tục khi người dùng nhấn nút UP/DOWN (sẽ làm hỏng Flash rất nhanh). Chỉ thực hiện Đọc/Ghi ở 2 thời điểm sau:

* **Quá trình ĐỌC (On Boot):**
  Trong hàm `main()`, trước khi vào vòng lặp `while(1)` và sau các hàm `MX_Init`, thay vì gọi trực tiếp `Init_Default_Intervals();`, hãy gọi hàm `Load_Settings_From_Flash();`.

* **Quá trình GHI (On Save & Exit):**
  Trong hàm `Process_Buttons()`, tại logic của **NÚT SETTING/RUN (PA8)**:
  Khi đang ở trạng thái Cài đặt (Current_UI_State khác `UI_STATE_MAIN`) và người dùng nhấn PA8 để thoát ra chạy lại từ đầu: 
  **Bổ sung hàm `Save_Settings_To_Flash();`** vào ngay trước khi chuyển state về `UI_STATE_MAIN` và trước khi thiết lập lại `Run_Hour`, `Run_Min`, `Run_Sec`.