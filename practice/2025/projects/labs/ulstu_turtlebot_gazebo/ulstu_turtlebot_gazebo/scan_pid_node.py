import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanPIDNode(Node):
    def __init__(self):
        super().__init__('laser_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription  # предотвращает удаление подписки
        self.get_logger().info("Лазерный сканер подключен. Ожидание данных...")

    def scan_callback(self, msg):
        """ Метод обработки данных лидара """
        ranges = msg.ranges  # Получаем массив расстояний до объектов
        num_readings = len(ranges)

        # Углы в LaserScan начинаются от `angle_min`, идут с шагом `angle_increment`
        left_angle = 90  # Угол слева (градусы)
        
        # Вычисляем индекс в массиве `ranges` для 90 градусов
        left_index = int((left_angle - msg.angle_min * 180.0 / 3.14159) / (msg.angle_increment * 180.0 / 3.14159))

        # Проверяем, есть ли данные в нужном индексе
        if 0 <= left_index < num_readings:
            left_distance = ranges[left_index]
            if left_distance == float('inf'):
                self.get_logger().info(f"Слева ({left_angle}°): нет препятствий")
            else:
                self.get_logger().info(f"Слева ({left_angle}°): {left_distance:.2f} м")
        else:
            self.get_logger().error("Ошибка: индекс угла вне диапазона!")

def main(args=None):
    rclpy.init(args=args)
    node = ScanPIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
