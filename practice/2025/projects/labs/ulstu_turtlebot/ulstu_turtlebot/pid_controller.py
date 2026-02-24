"""
PID Controller класс для управления роботом
"""


class PIDController:
    """
    PID контроллер для управления
    """
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=None, integral_limits=None):
        """
        Инициализация PID контроллера
        
        Args:
            kp (float): Пропорциональный коэффициент
            ki (float): Интегральный коэффициент
            kd (float): Дифференциальный коэффициент
            output_limits (tuple): Кортеж (min, max) для ограничения выходного сигнала
            integral_limits (tuple): Кортеж (min, max) для ограничения интеграла (anti-windup)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.output_limits = output_limits
        self.integral_limits = integral_limits if integral_limits else output_limits
        
        # Внутренние переменные
        self._prev_error = 0.0
        self._integral = 0.0
        self._first_run = True
        
    def compute(self, setpoint, measured_value, dt):
        """
        Вычисление выходного сигнала PID контроллера
        
        Args:
            setpoint (float): Желаемое значение
            measured_value (float): Измеренное значение
            dt (float): Временной шаг (в секундах)
            
        Returns:
            float: Управляющий сигнал
        """
        # Вычисление ошибки
        error = setpoint - measured_value
        
        # Пропорциональная составляющая
        p_term = self.kp * error
        
        # Интегральная составляющая с anti-windup
        self._integral += error * dt
        
        # Ограничение интеграла для предотвращения windup
        if self.integral_limits is not None:
            integral_value = self.ki * self._integral
            if integral_value > self.integral_limits[1]:
                self._integral = self.integral_limits[1] / self.ki if self.ki != 0 else 0
            elif integral_value < self.integral_limits[0]:
                self._integral = self.integral_limits[0] / self.ki if self.ki != 0 else 0
        
        i_term = self.ki * self._integral
        
        # Дифференциальная составляющая
        if self._first_run:
            derivative = 0.0
            self._first_run = False
        else:
            derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        
        d_term = self.kd * derivative
        
        # Суммарный управляющий сигнал
        output = p_term + i_term + d_term
        
        # Ограничение выхода
        if self.output_limits is not None:
            output = max(self.output_limits[0], min(output, self.output_limits[1]))
        
        # Сохранение текущей ошибки для следующей итерации
        self._prev_error = error
        
        return output
    
    def reset(self):
        """Сброс внутренних переменных контроллера"""
        self._prev_error = 0.0
        self._integral = 0.0
        self._first_run = True
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """
        Установка коэффициентов PID
        
        Args:
            kp (float, optional): Пропорциональный коэффициент
            ki (float, optional): Интегральный коэффициент
            kd (float, optional): Дифференциальный коэффициент
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
