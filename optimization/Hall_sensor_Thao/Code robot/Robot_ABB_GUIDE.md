# Hướng dẫn sử dụng và các lưu ý khi dùng Robot ABB

### Mỗi lần thu data cần đo noise của cảm biến trước
Cắm nguồn, kết nối UART với máy tính rồi đọc log rồi tính std mỗi cảm biến.

### Set up: 
1. Cắm nguồn (dây điện màu vàng)
2. Chờ màn hình của bảng điều khiển hiển thị
3. Cắm dây mạng màu trắng vào cổng Lan (IP: 192.168.125.1)

### Các thao tác trên màn hình điều khiển: 
1. Khi mới khởi động Robot lên, việc đầu tiên là phải Calib => Vào Calibration => Update => Ok => Hiện Caliberated hay Updated là được. 
2. Jogging -> Motion mode: Điều khiển robot bằng cách giữ nút bên sườn phải và điều khiển các trục 1,2,3 trước. 
3. Để robot ở độ cao vừa phải, cách quyển sách khoảng 40cm cho an toàn, cách mặt bàn đang đặt 3 cuộc dây đồng khoảng 30-40cm để tránh va chạm. 
4. Motion mode -> Linear: check x,y,z (note lại tọa độ hiện tại) rồi check q1, q2, q3, q4 đã đúng là (0, 0, 1 ,0) hoặc (0, 0, -1, 0) chưa. Nếu chưa: 

5. Chọn mục menu (3 gạch ở góc trái phía trên màn hình điều khiển) -> chọn Program Editor -> Debug -> PP to main (để phần code trỏ về hàm main)
6. Sau đó ấn giữ nút bên sườn phải bảng ĐK rồi ấn nút play (ký hiệu hình tam giác) , lưu ý vẫn luôn giữ nút bên sườn phải. 
Sau đó, ấn run code move_to_one_point.py (điều khiển robot đến 1 điểm cho trước) -> nhập tọa độ x,y,z hiện tại (đã note lại ở mục 2.4) rx,ry,rz thì có thể nhập random
Chạy code move_to_one_point.py xong thì phần góc xoay (q1, q2, q3, q4) được calib về đúng (check lại Jogging/ Motion mode/ Linear)

7. Calib xong, mới chỉnh lại hướng của thanh gá sao cho hướng đầu của thanh gá (phần lắp viên nang) hướng theo chiều dương Ox, tức là hướng về phía bên trái, theo phương song song với quyển sách.

Mạch cảm biến được đặt ở phần góc dưới của quyền sách sao cho cảm biến 1 (JP25) ở vị trí góc trái trên.

Trước khi thu data, cần xác định tọa độ của 25 cảm biến trước:
Để xác định tọa độ 25 con, ta cần đầu thanh gá ở vị trí của cảm biến 1 (vị trí góc trái trên của mạch) => có 2 cách: 1 là jogging điều khiển tay đến vị trí đó/ 2 là chạy code move_to_one_point.py 

Khuyến nghị cách làm: 
- Dùng jogging để di chuyển robot ra gần tới sensor1 trước, sau đó vào Motion mode/ Linear để note lại x,y,z hiện tại 
- Lại ấn giữ nút sườn phải của bảng ĐK rồi chạy code move_to_one_point rồi nhập tọa độ (dựa vào tọa độ đã note ở trên, nhưng chỉ khác 1 chút để tránh robot đi quá xa)
<WARNING: nếu robot mà có lỡ đi quá xa ngoài ý muốn thì ngay lập tức buông tay khỏi nút sườn bên phải của bảng ra để robot dừng di chuyển và khi nhập tọa độ thì chỉ nhập thay đổi 1 ít một để robot di chuyển từ từ>
- Nếu robot bị mất kết nối với máy tính thì quay lại bước 5 (quay lại hàm pp to main rồi mới chạy lại code move_to_one_point)

Khi robot đã ở vị trí ngay trên sensor 1 (tốt nhất là nên ở trên khoảng 1cm để tránh va chạm với mảng cảm biến), vào lại Linear để xem tọa độ hiện tại (x,y tương ứng x,y của sensor 1 còn z thì trừ đi 1 khoảng do robot đang ở trên 1 chút) => như vậy ta được tọa độ của cảm biến 1

Nhập tọa độ trên vào # Sensor array origin (first sensor position - top-left corner) trong Hall_sensor_array_coor.py => run code => sensor_positions.csv 


### Xác định vùng ROI (vùng hoạt động hình hộp chữ nhật)

Vùng hoạt động có độ cao từ 3.5cm đến 11cm so với mặt phẳng cảm biến (tức là lấy Zsensor + 3.5 -> Zsensor + 11)

Đến phần này, tốt nhất là bỏ mảng cảm biến ra để tránh va chạm
X min là mép góc bên phải của mảng, X max là mép góc bên trái của mảng -> di robot ra 2 vị trí này và lấy Xmin, Xmax (đkhien tay hoặc dùng code) -> đến vị trí nào thì vào Linear lấy tọa độ
Y min là mép cạnh trên của mảng, Y max là mép cạnh dưới của mảng 

Note lại các khoảng của X,Y,Z -> vùng ROI

### Thu dữ liệu theo các quỹ đạo <Note: xuyên suốt quá trình robot chạy, nút sườn bên phải bảng ĐK phải được giữ>
1. Thu quỹ đạo ngẫu nhiên: robot_ABB_random.py 150 điểm
Ở mục # RANDOM SPACE LIMIT, nhập Xmin, Xmax, Ymin, Ymax, Zmin, Zmax
Giữ nút ở bảng ĐK rồi chạy code cho robot thu.
2. Thu quỹ đạo Helix: robot_ABB_helical.py 150 điểm
Trong `robot_ABB_helical.py`, nhập trực tiếp điểm đầu tooltip vào `HELIX_START_MM = (Xstart, Ystart, Zstart)`. Để tâm helix nằm tại Sensor 13, chọn `Xstart = Xsensor13 + R_BAR_BASE`, `Ystart = Ysensor13`, `Zstart = Zsensor13 + 3.5 cm`. Thay đổi tuple sẽ tịnh tiến toàn bộ đường đi, giữ nguyên hình dạng và độ lệch tương đối.
Giữ nút ở bảng ĐK rồi chạy code cho robot thu.

3. Thu quỹ đạo Grid (dạng Lưới): robot_ABB_grid.py
Ở mục # START TOOL POSITION, nhập điểm khởi đầu là tọa độ của sensor 1 (góc trái trên của mảng)
Giữ nút ở bảng ĐK rồi chạy code cho robot thu. 

<WARNING: trong quá trình robot chạy quỹ đạo, nếu có biểu hiện robot chạy ngoài ý muốn thì phải ngay lập tức bỏ giữ nút sườn bên phải đề robot ngừng chạy, tránh robot va vào mảng cảm biến, mảng.>
Khi kết thúc session thu data, jogging đưa 6 trục về 0.0x độ (x tiểu học) rồi rút dây nguồn của robot.
