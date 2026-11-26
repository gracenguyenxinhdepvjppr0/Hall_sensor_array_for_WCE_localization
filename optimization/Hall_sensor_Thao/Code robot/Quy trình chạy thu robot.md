# Manual thu dữ liệu robot và mảng Hall

> Tài liệu này áp dụng cho các script `robot_ABB_*.py` trong thư mục này và giao thức UART `CAPTURE` hiện tại. Mỗi pose hợp lệ thu 5 lần quét, mỗi lần gồm 16 giá trị Bx, 16 By và 16 Bz. Script lấy trung bình 5 lần và lưu 48 kênh cùng một `sample_id` với tọa độ robot.

> **Đối chiếu tài liệu:** `DataCollection_Summary.md` còn mô tả một phiên bản thu cũ (25 sensor, UART binary 115200 baud và các tùy chọn CLI). Phần hướng dẫn hệ tọa độ/tool offset cho Robot Arm được tham khảo ở đó; số sensor, giao thức UART và lệnh chạy trong manual này lấy theo firmware và script hiện tại.

## 1. Kiểm tra trước khi bật hệ thống

### Firmware và mảng cảm biến

1. Build và nạp firmware `C:\Users\Admin\OneDrive\Documents\Hall_sensor_array\Git\Firmware\hall_sensor_array\Core\Src\main.c` có giao thức `CAPTURE` vào đúng board STM32.
2. Cấp nguồn cho STM32 và mảng cảm biến; kiểm tra dây SPI, nguồn và mass chung.
3. Nối UART đúng chiều: PC-TX → UART7-RX, UART7-TX → PC-RX, GND chung. Dùng mức logic UART 3.3 V; không đưa tín hiệu RS-232 hoặc 5 V vào chân MCU.
4. Cấu hình UART hiện tại: `COM5`, `921600 baud`, `8N1`, không flow control. Nếu Windows cấp số COM khác, sửa `UART_PORT` trong script trước khi chạy.

Firmware này **không phát dữ liệu liên tục khi rảnh**. Chỉ thấy dữ liệu sau khi gửi lệnh `CAPTURE`; trạng thái im lặng trên UART lúc khởi động là bình thường.

Độ dày sách kê ~ 4cm, độ cao của cảm biến trên mạch ~ 1,5cm. Tổng độ cao so với mặt bàn là ~6cm .
### Robot ABB

1. Bật controller, kiểm tra không có lỗi đang chặn chuyển động và bật robot theo quy trình an toàn tại cell.
2. Mở chương trình/server phía robot để nhận pose qua TCP. Các script đang dùng `192.168.125.1:2500`; máy tính phải nối đúng mạng robot.
3. Xác nhận server robot trả về một chuỗi dạng `[ ... ]` **sau khi robot đã tới pose**. Script hiện nhận bất kỳ nội dung nào nằm trong ngoặc vuông là tín hiệu đã tới điểm. Ở bước warm-up đầu, script chưa dừng khi thiếu phản hồi; hãy xác nhận robot đã tới vị trí khởi động trước khi để lượt thu tiếp tục.
4. Kiểm tra vùng làm việc, gá cảm biến, tool offset và điểm bắt đầu. Lần chạy đầu nên giảm tốc độ robot, đứng ngoài vùng chuyển động và sẵn sàng dừng bằng bộ FlexPendant.

### Sử dụng Robot Arm và kiểm tra pose

Phần robot trong `DataCollection_Summary.md` mô tả đúng nguyên tắc: PC gửi pose, chờ robot báo đã tới, rồi mới thu cảm biến. Các thông số dưới đây được đối chiếu với các script hiện tại:

1. **Kiểm tra hệ tọa độ và giới hạn quỹ đạo.** Tọa độ X/Y/Z trong script là millimeter. Mỗi quỹ đạo hiển thị pose đầu và các biên dạng độ lệch trên console; đối chiếu chúng với frame mà chương trình ABB nhận trước khi chạy.

   | Quỹ đạo | Pose đầu tooltip (mm) | Giới hạn tương đối từ pose đầu (mm) |
   |---|---|---|
   | Grid | `(-45, 685, -63.5)` | `dX: 0..140`, `dY: 0..140`, `dZ: 0..25` |
   | Helical | `(95, 760, -63.5)` | `dX: -140..0`, `dY: -70..70`, `dZ: 0..30` |
   | Lissajous | `(-60, 760, -63.5)` | `dX: 0..155`, `dY: -125..15`, `dZ: 0..30` |
   | Random | Chọn ngẫu nhiên đầu mỗi lượt | Tính từ workspace hiện có và in trên console |

   Để dời Helical, sửa tuple `HELIX_START_MM = (x, y, z)` trong `robot_ABB_helical.py` (tọa độ tooltip, mm). Giới hạn tương đối và hình dạng giữ nguyên; điểm đầu là tuple này và toàn bộ quỹ đạo tịnh tiến theo nó. Script in biên tooltip tuyệt đối ra console. Start hiện tại cho vùng X dự kiến `-45..95 mm`.

   Để dời Lissajous, chỉ sửa tuple `LISSAJOUS_START_MM = (x, y, z)` trong `robot_ABB_Lissajous Curve.py` (tọa độ tooltip, mm); `XC`, `YC`, `ZC` được lấy tự động từ tuple. Giới hạn tương đối và hình dạng quỹ đạo giữ nguyên, toàn bộ đường đi được tịnh tiến theo start mới; điểm đầu và cuối trùng nhau. Script kiểm tra start có ba tọa độ hữu hạn, kiểm tra các biên tương đối chứa offset 0 và in biên tooltip tuyệt đối ra console. Với start hiện tại, X chạy từ -60 đến 95 mm. Biên in ra không kiểm tra giới hạn khớp hoặc va chạm; xác nhận vùng làm việc trước khi chạy robot.
2. **Xác nhận tool offset.** Các script hiện dùng offset trong hệ tool `o = [-3.0, 0.0, 98.5] mm`. Quỹ đạo định nghĩa vị trí tooltip/nam châm `p_tip`, còn robot nhận TCP. Python tính:

   ```text
   p_TCP = p_tip - R(q) o
   ```

   Vì vậy sai tool offset, chiều lắp tool hoặc frame của ABB có thể làm đầu tool lệch khỏi vị trí dự kiến. Đối chiếu offset với tool thực tế và cấu hình TCP/WorkObject ở controller trước khi chạy.
3. **Xác nhận định dạng orientation.** Python gửi TCP qua TCP/IP theo dạng:

   ```text
   [x_mm,y_mm,z_mm];[qw,qx,qy,qz]
   ```

   Quaternion trên socket dùng thứ tự scalar-first `(qw,qx,qy,qz)`; không đổi thành thứ tự khác trong chương trình ABB. Orientation tham chiếu hiện tại trong các script là `(0,0,1,0)`; một số quỹ đạo tạo thêm orientation quanh hướng tham chiếu.
4. **Kiểm tra hướng nam châm.** Code giả định trục nam châm trong tool frame là `m_tool = [1,0,0]`, sau đó tính `m_world = R(q)m_tool` và lưu `mx,my,mz`. Xác nhận trục X của tool thực sự trùng chiều quy ước cực nam châm trước khi dùng các cột hướng trong CSV.
5. **Đặt tốc độ ở controller ABB.** Script Python chỉ gửi pose và chờ phản hồi; tốc độ, gia tốc, giới hạn khớp và WorkObject không được đặt trong script. Đặt các giá trị phù hợp ở chương trình/controller robot. Khi bắt đầu thu, giám sát chuyển động và thử ở tốc độ thấp.
6. **Xem trước đường đi nếu cần.** Có thể chạy `python .\visualize_trajectories.py` để mở đồ thị bốn quỹ đạo trong hệ tọa độ mm. Lệnh này không kết nối robot/UART và không mô phỏng TCP offset, giới hạn khớp hoặc va chạm; vẫn phải kiểm tra an toàn trên robot.

Sau khi gửi từng pose, Python chờ tối đa 20 giây cho phản hồi robot, rồi chờ thêm khoảng 2–2.5 giây để cơ khí ổn định trước khi yêu cầu STM32 đo. Server phía ABB cần chỉ gửi phản hồi trong ngoặc vuông sau khi chuyển động hoàn tất; code hiện tại chỉ kiểm tra có cặp ngoặc vuông, không xác thực nội dung ACK. Tọa độ trong CSV là pose tooltip đã ra lệnh, không phải vị trí encoder độc lập đọc từ robot.

### Máy tính và file đầu ra

Mở PowerShell trong thư mục `Code robot`. Kiểm tra Python và các thư viện được script dùng:

```powershell
python --version
python -m pip show numpy scipy pyserial
```

Nếu thiếu thư viện, cài trong đúng môi trường Python sẽ dùng để chạy:

```powershell
python -m pip install numpy scipy pyserial
```

Trước khi chạy, mở script tương ứng và kiểm tra `UART_PORT`, `UART_BAUD`, `ROBOT_IP`, `ROBOT_PORT`, `COORD_FILE` và `SENSOR_FILE`. Các đường dẫn đầu ra hiện được đặt như sau:

| Quỹ đạo | File tọa độ | File từ trường |
|---|---|---|
| Grid | `Grid_points_coordinates_2.csv` trong thư mục `Code robot` | `Grid_data_2.csv` trong thư mục `Code robot` |
| Helical | `Helix_points_coordinates_2.csv` trong thư mục `Code robot` | `Helix_data_2.csv` trong thư mục `Code robot` |
| Lissajous | `Lissajous_points_coordinates.csv` trong thư mục `Code robot` | `Lissajous_data.csv` trong thư mục `Code robot` |
| Random | `Random_points_coordinates.csv` trong thư mục `Code robot` | `Random_data.csv` trong thư mục `Code robot` |

Các script mở file ở chế độ ghi mới nên lần chạy tiếp theo có thể **ghi đè dữ liệu cũ**. Hãy đổi tên hai file đầu ra hoặc sao lưu dữ liệu cũ trước khi chạy. Không dùng `START_POINT` để tiếp tục phiên cũ nếu chưa tự xử lý việc nối file và `sample_id`; cách tiếp tục hiện chưa nhất quán giữa các script. Trong Helical và Lissajous, các dòng `START_POINT` hiện đang bị comment và không cắt quỹ đạo; hai script này bắt đầu từ pose đầu.

## 2. Kiểm tra log UART trước khi chạy robot

Thử UART riêng trước lần thu đầu tiên:

1. Mở một terminal UART có hỗ trợ 921600 baud, đặt `8N1`, tắt flow control và chọn đúng COM.
2. Chọn kết thúc dòng LF hoặc CRLF (không chọn CR-only), rồi gửi dòng ASCII sau:

   ```text
   CAPTURE,1,5
   ```

3. Với firmware mới, log đúng sẽ có dạng:

   ```text
   ACK,1,5
   Bx,1,1,<16 số nguyên>
   By,1,1,<16 số nguyên>
   Bz,1,1,<16 số nguyên>
   ...
   Bx,1,5,<16 số nguyên>
   By,1,5,<16 số nguyên>
   Bz,1,5,<16 số nguyên>
   DONE,1,5
   ```

   Mỗi dòng Bx/By/Bz có mã mẫu, số thứ tự lần quét và 16 giá trị. Giá trị UART là số nguyên µT; script đổi sang mT trước khi ghi CSV.
4. Nếu nhận `ERR,...`, không có `ACK`, thiếu dòng trục hoặc không có `DONE`, kiểm tra firmware đã được nạp đúng, dây TX/RX, GND, COM và baud.
5. Đóng hoàn toàn terminal UART trước khi chạy Python. Windows thường chỉ cho một chương trình mở `COM5` tại một thời điểm; nếu terminal còn giữ cổng, script sẽ báo lỗi mở serial.

Không chạy lệnh kiểm tra này cùng lúc với robot. Đây chỉ là phép thử UART riêng; khi chạy robot, Python tự gửi `CAPTURE` sau mỗi pose.

## 3. Chạy một quỹ đạo

Chỉ chạy **một** script tại một thời điểm. Trong PowerShell ở thư mục `Code robot`, chọn đúng một lệnh:

```powershell
python .\robot_ABB_grid.py
python .\robot_ABB_helical.py
python ".\robot_ABB_Lissajous Curve.py"
python .\robot_ABB_random.py
```

Tọa độ điểm đầu của các quỹ đạo: Với (x, y, z) là tọa độ sau khi calib quat
1. Grid: (x - 30, y - 30, z + 20) mm
2. Helical: (x - 45, y + 45, z + 20) mm 
3. Lissajous: (x - 45, y + 45, z + 20) mm 

Giới hạn vùng ROI: 
1. Helical: 
Trình tự chạy của script: 

1. Kết nối TCP tới robot và mở UART.
2. Di chuyển tới vị trí khởi động/warm-up.
3. Với từng sample: gửi pose, chờ phản hồi robot, chờ thêm settling delay trong script, rồi gửi `CAPTURE,<sample_id>,5` tới STM32.
4. Kiểm tra `ACK`, nhận đủ 5 bộ Bx/By/Bz đúng `sample_id` và đúng thứ tự 1–5, rồi kiểm tra `DONE`.
5. Tính trung bình 48 kênh, ghi một dòng tọa độ và một dòng từ trường có cùng `sample_id`, sau đó flush hai file.

### Ước tính thời gian thu mỗi pose

Thời gian một sample có thể tính gần đúng:

```text
T_sample ≈ T_robot_di_chuyển + T_settle + T_UART_capture + T_ghi_file
```

- `T_robot_di_chuyển` là thời gian từ lúc gửi pose đến khi ABB trả phản hồi. Code chờ tối đa 20 giây; 20 giây là timeout, không phải thời gian mặc định của mỗi điểm.
- `T_settle` được đặt trong script: Grid dùng 2.0 giây cho orientation đầu tiên tại mỗi vị trí và 2.4 giây cho 7 orientation còn lại; Helical, Lissajous và Random dùng 2.5 giây/sample.
- STM32 quét 5 lần. Comment timing trong firmware ước tính một `Scan16()` khoảng 4.4 ms; do đó 5 lần quét mất khoảng 22 ms. Cộng thời gian truyền UART và status, có thể lấy khoảng **25–30 ms/sample** làm ước tính ban đầu. Đây chưa phải số đo thực nghiệm; firmware hiện không in thời gian capture ra UART.
- Ghi/flush CSV thường nhỏ so với settling và chuyển động robot, nhưng phụ thuộc máy tính/ổ đĩa.

**Thời gian nền không tính robot di chuyển** (settling + khoảng 25–30 ms capture):

| Quỹ đạo | Số pose trong code | Thời gian nền ước tính |
|---|---:|---:|
| Helical | 150 | khoảng 6 phút 19 giây |
| Lissajous | 150 | khoảng 6 phút 19 giây |
| Random | 150 | khoảng 6 phút 19 giây |
| Grid | 384 vị trí × 8 orientation = 3.072 pose | khoảng 2 giờ 1 phút 36 giây đến 2 giờ 1 phút 51 giây |

Các tổng này **chưa gồm thời gian robot di chuyển giữa các pose**. Cộng thêm một lần di chuyển tới vị trí khởi động và 2 giây warm-up. Với Grid, mỗi vị trí không gian có 8 pose orientation; vì vậy riêng settling tại một vị trí là `2.0 + 7 × 2.4 = 18.8 giây`, chưa tính chuyển động và capture.

Để biết thời gian thực của hệ thống, đo từ lúc in `Sample n/...` đến lúc bắt đầu sample tiếp theo trong PowerShell ở một số điểm đại diện (điểm gần, xa và đổi orientation). Trung bình đó mới phản ánh tốc độ robot, ACK và thời gian truyền thực tế. Nếu gặp `Robot timeout`, script chờ đến giới hạn 20 giây, bỏ sample đó rồi tiếp tục nên tổng thời gian có thể tăng mà số dòng dữ liệu không tăng.

Theo dõi cửa sổ PowerShell. Các dấu hiệu bình thường gồm `Connected`, `UART connected: COM5 @ 921600`, thông tin `Sample n/...` và cuối lượt là `Scan finished` rồi `Disconnected`. Parser UART hiện không in toàn bộ dòng raw khi chạy; nó chỉ báo lỗi nếu handshake/frame sai. Muốn xem raw log, dùng terminal UART ở bước 2 trước khi chạy, rồi đóng terminal.

Sau mỗi pose được lưu thành công, bốn script in một dòng đo thời gian, ví dụ:

```text
[TIMING] sample_id=12 robot_wait=1.843s settle=2.501s uart=0.028s csv=0.003s total=4.378s
```

`robot_wait` gồm thời gian gửi lệnh và chờ robot báo tới; `settle` là thời gian chờ ổn định; `uart` gồm gửi lệnh CAPTURE, nhận đủ 5 lần quét và tính trung bình; `csv` gồm ghi và flush hai file. `total` đo từ trước lúc gửi pose tới sau khi flush CSV. Log thời gian hiện chỉ in trên console, không thêm cột vào CSV. Với timeout hoặc lỗi UART, sẽ in `status=ROBOT_TIMEOUT` hoặc `status=UART_ERROR` để biết pose đó chưa được lưu thành công.

Nếu thấy `Robot timeout`, Grid, Helical và Lissajous hiện bỏ qua sample; Random đã được chỉnh để dừng hẳn nếu timeout ở warm-up hoặc sample. Dừng và kiểm tra robot/server nếu robot chưa thật sự tới điểm; không nên tiếp tục thu khi vị trí không chắc chắn. Nếu thấy `[UART ERROR]`, script dừng lượt thu để tránh ghép nhầm tọa độ với dữ liệu cảm biến.

## 4. Kết thúc và kiểm tra dữ liệu

### Kết thúc bình thường

Chờ tới khi PowerShell in:

```text
Scan finished
Disconnected
```

`Disconnected` nghĩa là script đã đóng các file CSV, socket robot và cổng UART. Sau đó kiểm tra hai file đầu ra:

- File tọa độ có `sample_id`, `x`, `y`, `z`, hướng nam châm và các góc.
- File từ trường có `sample_id` và 48 cột: 16 Bx, tiếp theo 16 By, tiếp theo 16 Bz; giá trị là mT sau khi lấy trung bình 5 lần quét.
- Số dòng dữ liệu hai file phải bằng nhau; các `sample_id` phải ghép được một-một giữa hai file.
- Một sample có thể vắng mặt nếu robot timeout; `sample_id` bị khuyết không tự nó có nghĩa là UART ghép sai.

### Dừng giữa chừng

Script không gửi lệnh STOP riêng cho controller robot. Nếu cần dừng vì tình huống cơ khí, dừng chuyển động bằng bộ điều khiển ABB/E-stop theo quy trình cell trước; không dựa vào Ctrl+C để dừng chuyển động vật lý. Khi robot đã dừng an toàn, nhấn `Ctrl+C` trong PowerShell để kết thúc Python và đóng tài nguyên. Các dòng CSV đã ghi trước đó được flush sau mỗi sample hoàn tất.

## 5. Xử lý nhanh lỗi thường gặp

| Triệu chứng | Việc cần kiểm tra |
|---|---|
| Không kết nối được robot | Mạng/IP máy tính, `ROBOT_IP`, `ROBOT_PORT`, server phía robot đã chạy chưa. |
| Robot timeout | Phản hồi robot có đúng dạng `[ ... ]` và chỉ gửi sau khi đã tới điểm không; kiểm tra robot có bị pause hoặc báo lỗi không. |
| Không mở được COM | Đóng PuTTY/Tera Term/Serial Monitor, kiểm tra `UART_PORT`, dây USB-UART và quyền truy cập COM. |
| `No ACK` hoặc `No DONE` | Xác nhận đã nạp firmware CAPTURE, baud 921600, dây TX/RX đảo đúng chiều, GND chung và STM32 đang chạy. |
| `ERR,EXPECTED_CAPTURE_ID_5` | Firmware nhận lệnh sai định dạng; gửi đúng `CAPTURE,<id>,5` kết thúc bằng LF/Enter. |
| Không thấy UART tự phát | Bình thường khi firmware đang rảnh; gửi `CAPTURE` để kiểm tra. |
| Không thấy CSV hoặc file nằm chỗ khác | Xem `COORD_FILE`/`SENSOR_FILE` trong script đang chạy; sửa thành đường dẫn mong muốn trước lượt tiếp theo. |

