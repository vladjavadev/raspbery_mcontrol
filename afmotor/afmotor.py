"""
Adafruit Motor Shield library для Raspberry Pi
Портирована с оригинальной библиотеки Arduino
"""
try:
    import RPi.GPIO as GPIO
except:
    import Mock.GPIO as GPIO

import time

# Константы для команд моторов
FORWARD = 1
BACKWARD = 2
BRAKE = 3
RELEASE = 4

# Стили шагов для степперов
SINGLE = 1
DOUBLE = 2
INTERLEAVE = 3
MICROSTEP = 4

# Настройки микрошагов
MICROSTEPS = 16

# Таблица значений для микрошагов (синусоидальная кривая)
if MICROSTEPS == 8:
    MICROSTEPCURVE = [0, 50, 98, 142, 180, 212, 236, 250, 255]
elif MICROSTEPS == 16:
    MICROSTEPCURVE = [0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255]

# Пины GPIO для Raspberry Pi (настройте под ваше подключение)
# Эти пины должны соответствовать вашему подключению к 74HC595
MOTORLATCH = 2  # Пин защелки (ST_CP)
MOTORCLK = 3     # Пин тактового сигнала (SH_CP)
MOTORENABLE = 4  # Пин разрешения (OE)
MOTORDATA = 17    # Пин данных (DS)

# Биты для управления моторами в сдвиговом регистре
MOTOR1_A = 2
MOTOR1_B = 3
MOTOR2_A = 1
MOTOR2_B = 4
MOTOR3_A = 5
MOTOR3_B = 7
MOTOR4_A = 0
MOTOR4_B = 6

# PWM пины для моторов (используем аппаратный PWM если доступен)
MOTOR1_PWM = 18  # GPIO 18 (PWM0)
MOTOR2_PWM = 13  # GPIO 13 (PWM1)
MOTOR3_PWM = 19  # GPIO 19 (PWM1)
MOTOR4_PWM = 12  # GPIO 12 (PWM0)

# Частота PWM по умолчанию
PWM_FREQUENCY = 1600


class AFMotorController:
    """Контроллер для управления Adafruit Motor Shield"""
    
    def __init__(self):
        """Инициализация контроллера"""
        self.latch_state = 0
        self.initialized = False
        
    def enable(self):
        """Настройка GPIO пинов и инициализация"""
        if not self.initialized:
            # Настройка режима нумерации пинов
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Настройка пинов сдвигового регистра
            GPIO.setup(MOTORLATCH, GPIO.OUT)
            GPIO.setup(MOTORCLK, GPIO.OUT)
            GPIO.setup(MOTORENABLE, GPIO.OUT)
            GPIO.setup(MOTORDATA, GPIO.OUT)
            
            # Обнуление состояния
            self.latch_state = 0
            self.latch_tx()
            
            # Включение выходов (активный LOW)
            GPIO.output(MOTORENABLE, GPIO.LOW)
            
            self.initialized = True
            
    def latch_tx(self):
        """Передача байта данных в сдвиговый регистр 74HC595"""
        # Опускаем защелку
        GPIO.output(MOTORLATCH, GPIO.LOW)
        
        # Обнуляем линию данных
        GPIO.output(MOTORDATA, GPIO.LOW)
        
        # Передаем 8 бит (от старшего к младшему)
        for i in range(8):
            # Опускаем тактовый сигнал
            GPIO.output(MOTORCLK, GPIO.LOW)
            
            # Устанавливаем бит данных
            if self.latch_state & (1 << (7 - i)):
                GPIO.output(MOTORDATA, GPIO.HIGH)
            else:
                GPIO.output(MOTORDATA, GPIO.LOW)
            
            # Поднимаем тактовый сигнал (защелкиваем бит)
            GPIO.output(MOTORCLK, GPIO.HIGH)
        
        # Поднимаем защелку (переносим данные на выходы)
        GPIO.output(MOTORLATCH, GPIO.HIGH)


# Глобальный экземпляр контроллера
_controller = AFMotorController()


class AF_DCMotor:
    """Класс для управления DC мотором"""
    
    def __init__(self, motor_num: int, freq: int = PWM_FREQUENCY):
        """
        Инициализация DC мотора
        
        Args:
            motor_num: Номер мотора (1-4)
            freq: Частота PWM в Гц
        """
        if motor_num not in [1, 2, 3, 4]:
            raise ValueError("Номер мотора должен быть от 1 до 4")
            
        self.motornum = motor_num
        self.pwm_freq = freq
        self.pwm = None
        
        # Инициализация контроллера
        _controller.enable()
        
        # Определение битов управления и PWM пина
        if motor_num == 1:
            self.pin_a = MOTOR1_A
            self.pin_b = MOTOR1_B
            self.pwm_pin = MOTOR1_PWM
        elif motor_num == 2:
            self.pin_a = MOTOR2_A
            self.pin_b = MOTOR2_B
            self.pwm_pin = MOTOR2_PWM
        elif motor_num == 3:
            self.pin_a = MOTOR3_A
            self.pin_b = MOTOR3_B
            self.pwm_pin = MOTOR3_PWM
        else:  # motor_num == 4
            self.pin_a = MOTOR4_A
            self.pin_b = MOTOR4_B
            self.pwm_pin = MOTOR4_PWM
        
        # Обнуление битов мотора
        _controller.latch_state &= ~(1 << self.pin_a) & ~(1 << self.pin_b)
        _controller.latch_tx()
        
        # Настройка PWM
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_freq)
        self.pwm.start(0)
        
    def run(self, command: int):
        """
        Управление направлением вращения мотора
        
        Args:
            command: FORWARD, BACKWARD, BRAKE или RELEASE
        """
        if command == FORWARD:
            # A = 1, B = 0
            _controller.latch_state |= (1 << self.pin_a)
            _controller.latch_state &= ~(1 << self.pin_b)
            _controller.latch_tx()
            
        elif command == BACKWARD:
            # A = 0, B = 1
            _controller.latch_state &= ~(1 << self.pin_a)
            _controller.latch_state |= (1 << self.pin_b)
            _controller.latch_tx()
            
        elif command == RELEASE:
            # A = 0, B = 0 (свободное вращение)
            _controller.latch_state &= ~(1 << self.pin_a)
            _controller.latch_state &= ~(1 << self.pin_b)
            _controller.latch_tx()
            
        elif command == BRAKE:
            # A = 1, B = 1 (торможение)
            _controller.latch_state |= (1 << self.pin_a)
            _controller.latch_state |= (1 << self.pin_b)
            _controller.latch_tx()
    
    def set_speed(self, speed: int):
        """
        Установка скорости мотора через PWM
        
        Args:
            speed: Скорость от 0 до 255
        """
        speed = max(0, min(255, speed))  # Ограничение диапазона
        duty_cycle = (speed / 255.0) * 100.0
        self.pwm.ChangeDutyCycle(duty_cycle)
    
    def cleanup(self):
        """Очистка ресурсов"""
        if self.pwm:
            self.pwm.stop()

# Пример использования
if __name__ == "__main__":
    try:
        print("Тест DC мотора...")
        # motor4 = AF_DCMotor(4)
        motor3 = AF_DCMotor(3)
        motor2 = AF_DCMotor(2)
        # motor1 = AF_DCMotor(1)
        
        motor3.set_speed(200)
        # motor4.set_speed(150)
        # motor1.set_speed(200)
        motor2.set_speed(200)

        motor3.run(FORWARD)
        # motor4.run(FORWARD)
        # motor1.run(FORWARD)
        motor2.run(FORWARD)

        time.sleep(2)
        
        motor3.run(BACKWARD)
        # motor4.run(BACKWARD)
        # motor1.run(BACKWARD)
        motor2.run(BACKWARD)
        time.sleep(2)
        
        motor3.run(RELEASE)
        # motor4.run(RELEASE)
        # motor1.run(RELEASE)
        motor2.run(RELEASE)
        motor3.cleanup()
        # motor4.cleanup()
        # motor1.cleanup()
        motor2.cleanup()
        
    finally:
        motor3.cleanup()
        # motor4.cleanup()
        # motor1.cleanup()
        motor2.cleanup()
        print("\nЗавершено!")