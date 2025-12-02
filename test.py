import RPi.GPIO as GPIO
import time

# Номер піна в BCM-номерації (наприклад GPIO18 підтримує апаратний PWM)
PWM_PIN = 18

# Налаштування GPIO
GPIO.setmode(GPIO.BCM)       # Використовуємо BCM-нумерацію
GPIO.setup(PWM_PIN, GPIO.OUT)

# Створюємо PWM-об'єкт з частотою 1000 Гц
pwm = GPIO.PWM(PWM_PIN, 1000)
pwm.start(0)  # Починаємо з duty cycle = 0%

try:
    while True:
        # Плавне збільшення яскравості (наприклад для LED)
        for duty in range(0, 250, 10):
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.5)

        # Плавне зменшення
        for duty in range(100, -1, -5):
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.1)

except KeyboardInterrupt:
    pass

# Завершення роботи
pwm.stop()
GPIO.cleanup()