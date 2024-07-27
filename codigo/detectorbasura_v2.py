import cv2
import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera

# Configurar GPIO
plastic_servo_pin = 17
glass_servo_pin = 27
cardboard_servo_pin = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(plastic_servo_pin, GPIO.OUT)
GPIO.setup(glass_servo_pin, GPIO.OUT)
GPIO.setup(cardboard_servo_pin, GPIO.OUT)

plastic_servo = GPIO.PWM(plastic_servo_pin, 50)  # Canal PWM con 50Hz
glass_servo = GPIO.PWM(glass_servo_pin, 50)
cardboard_servo = GPIO.PWM(cardboard_servo_pin, 50)

# Iniciar servos en la posición inicial
plastic_servo.start(0)
glass_servo.start(0)
cardboard_servo.start(0)

# Cargar los modelos XML
fullbody_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')
upperbody_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_upperbody.xml')
plastic_bottle_cascade = cv2.CascadeClassifier('plastic_bottle.xml')
glass_bottle_cascade = cv2.CascadeClassifier('glass_bottle.xml')
cardboard_cascade = cv2.CascadeClassifier('cardboard.xml')

# Inicializar la cámara
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)  # Dejar que la cámara se caliente

def detect_and_act():
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detección de personas
        fullbody = fullbody_cascade.detectMultiScale(gray, 1.1, 4)
        upperbody = upperbody_cascade.detectMultiScale(gray, 1.1, 4)

        person_detected = False
        for (x, y, w, h) in fullbody:
            cv2.rectangle(image, (x, y), (x+w, y+h), (255, 255, 0), 2)
            person_detected = True
        for (x, y, w, h) in upperbody:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)
            person_detected = True

        # Si se detecta una persona, proceder con la detección de botellas y cartones
        if person_detected:
            plastic_bottles = plastic_bottle_cascade.detectMultiScale(gray, 1.3, 5)
            glass_bottles = glass_bottle_cascade.detectMultiScale(gray, 1.3, 5)
            cardboards = cardboard_cascade.detectMultiScale(gray, 1.3, 5)

            for (x, y, w, h) in plastic_bottles:
                cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                plastic_servo.ChangeDutyCycle(7)  # Mover servo para botellas de plástico
                time.sleep(1)
                plastic_servo.ChangeDutyCycle(0)  # Detener servo
            for (x, y, w, h) in glass_bottles:
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                glass_servo.ChangeDutyCycle(7)  # Mover servo para botellas de vidrio
                time.sleep(1)
                glass_servo.ChangeDutyCycle(0)  # Detener servo
            for (x, y, w, h) in cardboards:
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cardboard_servo.ChangeDutyCycle(7)  # Mover servo para cartón
                time.sleep(1)
                cardboard_servo.ChangeDutyCycle(0)  # Detener servo

        # Mostrar frame con detecciones
        cv2.imshow('Frame', image)
        rawCapture.truncate(0)  # Limpiar el stream para el siguiente frame

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Ejecutar detección y acción
try:
    detect_and_act()
finally:
    cv2.destroyAllWindows()
    plastic_servo.stop()
    glass_servo.stop()
    cardboard_servo.stop()
    GPIO.cleanup()