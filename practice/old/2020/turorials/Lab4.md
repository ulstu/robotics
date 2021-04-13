# Лабораторная работа №4

## Подготовка мира gazebo

За основу берётся мир AutoRace. Запустить его можно с помощью следующей команды:

```bash
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```

С помощью редактора моделей создаётся фигура требуемой формы. Нажав правой кнопкой можно указать материал модели. Ссылка на список материалов и цветов находится в секции "Полезные ссылки" . 

Функция вставки позволяет добавить созданную ранее фигуру в мир. После добавления достаточного количества моделей разных цветов следует сохранить мир в удобное место.

Данный мир предполагает использование вариации робота burger. Чтобы его добавить, используется следующие параметры в файле запуска:

```
  <param name="robot_description" command="$(find xacro)/xacro $(find turtlebot3_description)/urdf/turtlebot3_burger_for_autorace.urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf" args="-urdf -model turtlebot3_burger -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description" />
```

## Получение изображения дороги

Получение несжатого изображение из камеры:

```
self.image_subscriber = rospy.Subscriber("camera/image", Image, self.image_callback)
```

Преобразование изображения из ROS Image в формат для работы с OpenCV:

```
def image_callback(self, image):
    self.bridge = cv_bridge.CvBridge()
    cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
```

Проективное преобразование изображение с целью выделения дороги:

```
def image_projection(image):
    height, width, _ = image.shape

    pts_src = numpy.array([[
        [width / 2 - width / 6, height / 2 + height / 13],  # Верхняя левая точка
        [width / 2 + width / 6, height / 2 + height / 13],  # Верхняя правая точка
        [width - width / 5, height],                        # Нижняя правая точка
        [width / 5, height],                                # Нижняя левая точка
    ]], numpy.int32)
    pts_dst = numpy.array([[200, 0], [800, 0], [800, 600], [200, 600]])

    h, _ = cv2.findHomography(pts_src, pts_dst)

    return cv2.warpPerspective(image, h, (1000, 600))
```

Здесь из исходного изображения вырезается область дороги, представляемая трапецией, и преобразуется в проективное изображение.

Трапеция имеет несколько параметров:

* Увеличение параметров width / 6 уменьшает длину верхнего основания
* Увеличение параметров width / 5 увеличивает длину основания
* Увеличение параметров height / 13 увеличивает длину высоты

Пример преобразования из реальной жизни:

![](https://marcosnietoblog.files.wordpress.com/2014/02/sample.png?w=640)

## Обнаружение дороги
### Способ 1

Из изображения дороги выделяются по цвету левая и правая линии:

```
def select_left_line(image):
    lower_yellow = numpy.array([22, 93, 0])
    upper_yellow = numpy.array([45, 255, 255])

    return cv2.inRange(image, lower_yellow, upper_yellow)

def select_right_line(image):
    lower_white = numpy.array([0, 0, 180])
    upper_white = numpy.array([25, 36, 255])

    return cv2.inRange(image, lower_white, upper_white)
```

После чего найти дорогу можно либо используя преобразования Хафа, либо с помощью использования метода скользящего окна для построения полинома второй степени.

Для построения полинома одной из линий сначала необходимо построить гистограмму:

```
histogram = numpy.sum(image, axis=0)
```

После чего находится позиция текущего окна:

```
if left_or_right == 'left':
    lane_base = numpy.argmax(histogram[:center])
elif left_or_right == 'right':
    lane_base = numpy.argmax(histogram[center:]) + center
```

Гистограмма разделяется на определённое количество окон и для каждого окна находится наилучшая линия:

```
# Нахождение границ окна по x и y
win_y_low = image.shape[0] - (window + 1) * window_height
win_y_high = image.shape[0] - window * window_height
win_x_low = x_current - window_width
win_x_high = x_current + window_width

# Нахожденение ненулевых пикселе в окне
good_lane_indices = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                     (nonzero_x >= win_x_low) & (nonzero_x < win_x_high)).nonzero()[0]
lane_indices.append(good_lane_indices)

# Удовлетворяет ли длина линии минимальной
if len(good_lane_indices) > pixel_threshold:
    x_current = numpy.int(numpy.mean(nonzero_x[good_lane_indices]))
```

Из полученных индексов линии можно построить полином:

```
lane_indices = numpy.concatenate(lane_indices)
x = nonzero_x[lane_indices]
y = nonzero_y[lane_indices]

if len(x) > 0 and len(y) > 0:
    lane_fit = numpy.polyfit(y, x, 2)
else:
    return

plot_y = numpy.linspace(0, image.shape[0] - 1, image.shape[0])

return lane_fit[0] * plot_y ** 2 + lane_fit[1] * plot_y + lane_fit[2]
```

Найдя полином для обоих линий можем найти центральную линию:

```
left_line_pts = numpy.array([numpy.transpose(numpy.vstack([left, plot_y]))])
right_line_pts = numpy.array([numpy.flipud(numpy.transpose(numpy.vstack([right, plot_y])))])

center = numpy.mean([left, right], axis=0)
pts = numpy.hstack((left_line_pts, right_line_pts))
center_line = numpy.array([numpy.transpose(numpy.vstack([center, plot_y]))])
cv2.fillPoly(image, numpy.int_([pts]), (0, 255, 0))
```

### Способ 2

Также можно обнаружить дорогу и посчитать расстояния до линий.

Для этого сначала необходимо наложить нужную маску. (Пример есть в первом способе) А далее выделить контуры с помощью:

```
_, cnts, _ = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
```

Где cnts - лист с точками контура.

Далее каждый контур можно вписать в прямоугольник минимальной площадью и получить его точки.

```
 for cnt in cnts:
     rect = cv2.minAreaRect(cnt)
     box = cv2.boxPoints(rect)
     box = np.int0(box)
     A, B, C, D = box
```

В A,B,C,D будут находиться координаты соответствующих точек.

Далее можно определять то, что это линия по ее пропорциям и считать расстояние до нее от центра изображения, зная его ширину.


## Обнаружение объектов

### Подготовка изображения

Для обнаружения объектов нужного цвета требуется перевести изображение в формат hsv:

```
hsv_target = cv2.cvtColor(source_image, cv2.COLOR_BGR2HSV)
```

После чего следует подобрать hsv-границы для обнаружения требуемого цвета с помощью сайта из секции "Полезные ссылки". Задав границы, можно выделить объекты лишь нужного цвета. 

Пример для выделения объектов фиолетового цвета:

```
def select_target_objects(image):
    lower_purple = numpy.array([140, 10, 0])
    upper_purple = numpy.array([160, 255, 255])
    return cv2.inRange(image, lower_purple, upper_purple)
```

Для более точного обнаружения объектов желательно размыть полученное изображение и избавить его от шумов:

```
target_mask = cv2.blur(target_mask, (9, 9), 3)
target_mask = cv2.erode(target_mask, None, iterations=2)
target_mask = cv2.dilate(target_mask, None, iterations=2)
```

### Нахождение объектов

Обнаружение шаров можно выполнить с помощью команды:

```
detected_circles = cv2.HoughCircles(target_mask, cv2.HOUGH_GRADIENT, 0.9, 50,
                                    param1=100, param2=55, minRadius=0, maxRadius=500)
```

В остальных случаях придётся использовать поиск контуров:

```py
edged = cv2.Canny(target_mask, 75, 200)
contours = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
```

## Отслеживание объектов

Для отслеживания объектов следует хранить два словаря: содержащий центры найденных объектов и содержащий количество кадров, пройденных с потери объекта из области видимости.

При каждом вызове функции обнаружения объектов осуществляется проверка найденных ранее объектов. Если были найдены шары, сохраняются координаты их центров.

```
for (x, y, r) in numpy.array(on_screen_objects[0, :]):
    on_screen_centroids.append((x, y))
```

 Если в данный момент нет отслеживаемых объектов, то они добавляются в список отслеживаемых:

```
for centroid in on_screen_centroids:
    self.add_object(centroid)
```

Иначе находятся расстояния от найденных шаров до отслеживаемых:

```
distances = dist.cdist(all_centroids, on_screen_centroids)
min_rows = distances.min(axis=1).argsort()
min_columns = distances.argmin(axis=1)[min_rows]
```

Для каждого полученного минимального расстояния обновляется координата центра и сбрасывается счётчик, отвечающий за количество кадров, с момента, когда объект пропал из области видимости:

```
for (row, column) in zip(min_rows, min_columns):
    if row in used_rows or column in used_columns:
        continue

    object_id = all_ids[row]
    self.on_screen[object_id] = on_screen_centroids[column]
    self.disappeared[object_id] = 0

    used_rows.add(row)
    used_columns.add(column)
```

 Если количество отслеживаемых объектов больше или равно, чем количество найденных, то для каждого неиспользованного расстояния увеличивается счётчик пропажи. Если он превысит необходимое количество, то объект удаляется из отслеживаемых. Если же количество найденных объектов оказалось больше, чем отслеживаемых, то все они добавляются в список отслеживаемых:

```
if distances.shape[0] >= distances.shape[1]:
    for row in unused_rows:
        object_id = all_ids[row]
        self.disappeared[object_id] += 1

        if self.disappeared[object_id] > self.delay:
            self.remove_object(object_id)
else:
    for column in unused_columns:
        self.add_object(on_screen_centroids[column])
```

## Полезные ссылки

Список доступных материалов в gazebo - http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

Сайт для подбора hsv-цветов - https://alloyui.com/examples/color-picker/hsv.html

Обнаружение дороги - https://medium.com/@mithi/advanced-lane-finding-using-computer-vision-techniques-7f3230b6c6f2

Отслеживание объектов - https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/
