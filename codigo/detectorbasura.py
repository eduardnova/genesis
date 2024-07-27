import cv2
import RPi.GPIO as GPIO
import time

# Configurar GPIO
servo_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
servo = GPIO.PWM(servo_pin, 50)  # Canal PWM con 50Hz

# Iniciar servo en la posicion inicial de giro que es 0
servo.start(0)

# Cargar los modelos XML
plastic_bottle_cascade = cv2.CascadeClassifier('plastic_bottle.xml')
glass_bottle_cascade = cv2.CascadeClassifier('glass_bottle.xml')
cardboard_cascade = cv2.CascadeClassifier('cardboard.xml')

# Inicializar la camara para capturar en video entiempo real
cap = cv2.VideoCapture(0)

def detect_and_act():
    while True:
        # Capturar frame
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detección
        plastic_bottles = plastic_bottle_cascade.detectMultiScale(gray, 1.3, 5)
        glass_bottles = glass_bottle_cascade.detectMultiScale(gray, 1.3, 5)
        cardboards = cardboard_cascade.detectMultiScale(gray, 1.3, 5)
        
        # Dibujar rectángulos y accionar servo si se detecta algo : botella, cartos o botella de cristar
        for (x, y, w, h) in plastic_bottles:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            servo.ChangeDutyCycle(7)  # Mover servo
            time.sleep(1)
            servo.ChangeDutyCycle(0)  # Detener servo
        for (x, y, w, h) in glass_bottles:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            servo.ChangeDutyCycle(7)
            time.sleep(1)
            servo.ChangeDutyCycle(0)
        for (x, y, w, h) in cardboards:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            servo.ChangeDutyCycle(7)
            time.sleep(1)
            servo.ChangeDutyCycle(0)
        
        # Mostrar frame con detecciones de los objectos ene ste caso los muestra en la pantalla con un cuadro indicando que lo reconoce
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Ejecutar detección y acción
try:
    detect_and_act()
finally:
    cap.release()
    cv2.destroyAllWindows()
    servo.stop()
    GPIO.cleanup()