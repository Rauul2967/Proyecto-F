import time
from time import sleep
from nanpy import (ArduinoApi, SerialManager)
import cv2
import numpy as np

# import math
# import serial
# import servo
# from pyfirmata import (Arduino, SERVO)
# from Arduino import *
# board = Arduino('9600',port="/dev/ttyUSB0")

try:
    conexion = SerialManager()  # device='/dev/ttyUSB0')
    a = ArduinoApi(conexion)  # = conexion)
except:
    print("fallo conexion arduino")

# SERVO
servoPin = 3  # pin analogo 3, util para el servo (PWM)

a.pinMode(servoPin, a.OUTPUT)  # pin toma valores de 0 a 255

# MOTORES

# MOTOR A
ENA = 5  # ENABLE A para pin analogo
IN1 = 2  # ENTRADAS DE PINES MOTOR A
IN2 = 4

# MOTOR B
ENB = 6  # ENABLE B para pin analogo
IN3 = 7  # ENTRADAS DE PINES MOTOR B
IN4 = 13

a.pinMode(ENA, a.OUTPUT)
a.pinMode(ENB, a.OUTPUT)
a.pinMode(IN1, a.OUTPUT)
a.pinMode(IN2, a.OUTPUT)
a.pinMode(IN3, a.OUTPUT)
a.pinMode(IN4, a.OUTPUT)

"""#SENSOR
a.pinMode(S0, a.OUTPUT)
a.pinMode(S1, a.OUTPUT)
a.pinMode(S2, a.OUTPUT)
a.pinMode(S3, a.OUTPUT)
a.pinMode(OUT, a.INPUT)
a.digitalWrite(S0, a.HIGH)
a.digitalWrite(S1, a.HIGH)"""


# funciones para el servo
def servo_uno(valores):
    a.analogWrite(servoPin, valores)
    time.sleep(0.0005)


def abrirServo():
    for i in range(0, 250, 1):
        servo_uno(i)


def cerrarServo():
    for j in range(250, 0, -1):
        servo_uno(j)


# funciones para accionar motores
def Adelante():
    a.digitalWrite(IN1, a.HIGH)
    a.digitalWrite(IN2, a.LOW)
    a.analogWrite(ENA, 255)

    a.digitalWrite(IN3, a.HIGH)
    a.digitalWrite(IN4, a.LOW)
    a.analogWrite(ENB, 255)


def Atras():
    a.digitalWrite(IN1, a.LOW)
    a.digitalWrite(IN2, a.HIGH)
    a.analogWrite(ENA, 255)

    a.digitalWrite(IN3, a.LOW)
    a.digitalWrite(IN4, a.HIGH)
    a.analogWrite(ENB, 255)


def Izquierda():
    a.digitalWrite(IN1, a.HIGH)
    a.digitalWrite(IN2, a.LOW)
    a.analogWrite(ENA, 255)

    a.digitalWrite(IN3, a.LOW)
    a.digitalWrite(IN4, a.HIGH)
    a.analogWrite(ENB, 255)


def Derecha():
    a.digitalWrite(IN1, a.LOW)
    a.digitalWrite(IN2, a.HIGH)
    a.analogWrite(ENA, 255)

    a.digitalWrite(IN3, a.HIGH)
    a.digitalWrite(IN4, a.LOW)
    a.analogWrite(ENB, 255)


def Parar():
    a.digitalWrite(IN1, a.LOW)
    a.digitalWrite(IN2, a.LOW)
    a.analogWrite(ENA, 0)

    a.digitalWrite(IN3, a.LOW)
    a.digitalWrite(IN4, a.LOW)
    a.analogWrite(ENB, 0)


# funcion atrapar a distancia minima, avanza y cierra servo para evitar que salgan las pelotitas
def AtraparPelota():
    sleep(0.1)
    abrirServo()
    sleep(0.5)
    Adelante()
    sleep(0.5)
    cerrarServo()
    Parar()
    sleep(0.5)


# funcion que almacena las pelotitas en la base
def SoltarPelotas():
    sleep(0.1)
    Adelante()
    sleep(0.5)
    Parar()
    sleep(0.05)
    abrirServo()
    sleep(0.5)
    Atras()
    sleep(0.5)
    cerrarServo()
    Parar()
    sleep(0.05)


"""buscaderecha = True #buscar a la derecha sino izquierda
retroceso = 0 #cantidad de veces que tiene que retroceder para buscar
def BuscarPelotas():
    sleep(1)
    global buscaderecha
    global retroceso
    if(buscaderecha):
        for i in range(retroceso):
            Derecha()
            sleep(0.2)
            Parar()
            sleep(0.05)
        Derecha()
        sleep(0.2)
        Parar()
        sleep(0.05)
        buscaderecha = False
    else:
        for i in range(retroceso):
            Izquierda()
            sleep(0.2)
            Parar()
            sleep(0.05)
        Izquierda()
        sleep(0.2)
        Parar()
        sleep(0.05)
        buscaderecha = True
    retroceso+=1"""

# captura de imagen de opencv, y definimos dimensiones pantalla
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

""" FILTROS DE AMARILLO  """
low_amarillo = np.array([15, 100, 80], np.uint8)  # 30,100,100
high_amarillo = np.array([25, 230, 250], np.uint8)  # 50,255,255

low_amarillo2 = np.array([20, 100, 80], np.uint8)  # 40,100,80
high_amarillo2 = np.array([30, 240, 255], np.uint8)  # 60,255,255

""" FILTROS DE ROJO  """
low_rojo = np.array([161, 100, 84], np.uint8)
high_rojo = np.array([179, 255, 255], np.uint8)

low_rojo2 = np.array([169, 155, 100], np.uint8)
high_rojo2 = np.array([189, 255, 255], np.uint8)

""" X,Y BASE  """
# X = para centralizar // Y = distancia decente a la camara

minimoX = 260
maximoX = 320

minimoY = 160
maximoY = 220

""" X,Y BOLA  """
# rangos: X = minimo 35, maximo 551 // Y = minimo 207, maximo 344

minimoXbola = 250
maximoXbola = 420

minimoYbola = 330
maximoYbola = 460

# las coordenadas se verifican respecto a la camara en X e Y
# por perspectiva los lados seran al reves (la derecha es la izquierda)

"""            X,Y BASE         """


def verificarCoordenadasY(valory):
    global minimoY
    global maximoY

    if (valory < minimoY):
        # avanza
        print("delante base")
        Adelante()
        sleep(0.5)
        Parar()
        sleep(0.05)
        return False
    if (valory > maximoY):
        # retrocede
        print("atras base")
        Atras()
        sleep(0.5)
        Parar()
        sleep(0.05)
        return False
    if (valory < maximoY and valory > minimoY):
        print("centrado Y")
        return True


def verificarCoordenadasX(valorx):
    global minimoX
    global maximoX

    if (valorx < minimoX and valorx < maximoX):
        # endereza a la izquierda
        print("izquierda base")
        Izquierda()
        sleep(0.25)
        Parar()
        sleep(0.05)
        return False
    if (valorx < maximoX and valorx > minimoX):
        print("centrado X")  # deteccion centro
        return True
    if (valorx > maximoX and valorx > minimoX):
        # endereza a la derecha
        print("derecha base")
        Derecha()
        sleep(0.25)
        Parar()
        sleep(0.05)
        return False


"""           X,Y BOLA           """


def verificarCoordenadasY2(valory):
    global minimoYbola
    global maximoYbola

    if (valory < minimoYbola):
        # avanza
        ("adelante")
        Adelante()
        sleep(0.5)
        Parar()
        sleep(0.05)
        return False
    if (valory > maximoYbola):
        # retrocede
        print("atras")
        Atras()
        sleep(0.5)
        Parar()
        sleep(0.05)
        return False
    if (valory < maximoYbola and valory > minimoYbola):
        # acomoda ya sea para adelante o atras para la captura
        print("centrado Y")
        return True


def verificarCoordenadasX2(valorx):
    global minimoXbola
    global maximoXbola

    if (valorx < minimoXbola and valorx < maximoXbola):
        # endereza a la izquierda
        print("izquierda")
        Izquierda()
        sleep(0.25)
        Parar()
        sleep(0.05)
        return False
    if (valorx < maximoXbola and valorx > minimoXbola):
        print("centrado X")  # deteccion centro
        return True
    if (valorx > maximoXbola and valorx > minimoXbola):
        # endereza a la derecha
        ("derecha")
        Derecha()
        sleep(0.25)
        Parar()
        sleep(0.05)
        return False


# funciones para motores que ayudan a buscar siguiente objetivo (pelotita o base)
def buscarObjetivo():
    for i in range(5):
        Izquierda()
        sleep(0.25)
        Parar()
        sleep(0.05)


def buscarObjetivo2():
    for i in range(6):
        Izquierda()
        sleep(0.25)
        Parar()
        sleep(0.05)


def encontrarBase():
    print("INICIANDO BUSQUEDA BASE")

    # variables definidas anteriormente como limites
    global minimoX
    global maximoX

    global minimoY
    global maximoY

    # almacena ultima posicion como auxiliares
    xbase = 0
    ybase = 0
    # booleano auxiliar para la deteccion
    detectado = False

    # verifican si cumple con las coordenadas
    enX = False
    enY = False

    # permite detener la funcion cuando termina de almacenar en la base
    almacenado = False
    while True:
        # lectura de video
        ret, fram2 = cap.read()
        # dimensiones que tiene el frame
        h, w, m = fram2.shape

        # convertir el frame de BGR a HSV
        hsv2 = cv2.cvtColor(fram2, cv2.COLOR_BGR2HSV)
        # busca los rangos de colores que se encuentran en el frame
        mark3 = cv2.inRange(hsv2, low_rojo, high_rojo)
        mark4 = cv2.inRange(hsv2, low_rojo2, high_rojo2)
        # combinar ambos rangos de colores para una mejor deteccion
        markbase = cv2.add(mark3, mark4)

        # busca contornos que cumpla con los rangos de colores
        _, cnts2, _ = cv2.findContours(markbase, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # ciclo que analiza todos los contornos que encuentra
        for cc in cnts2:
            # area del contorno a analizar (pixeles)
            area2 = cv2.contourArea(cc)

            # precision de aproximacion para deteccion
            epsilon2 = 0.02 * cv2.arcLength(cc, True)
            # verifica el tipo de contorno
            approx2 = cv2.approxPolyDP(cc, epsilon2, True)

            # si cumple con condiciones, debe ser la base
            if (area2 > 200 and len(approx2) == 4):
                # calcula coordenadas y tamaño del contorno
                x2, y2, w2, h2 = cv2.boundingRect(approx2)  # c
                # dibuja rectangulo en el frame
                cv2.rectangle(fram2, (x2, y2), (x2 + w2, y2 + h2), (0, 0, 255), 3)

                # almacenamos valores en x e y (divido en 2 para valor central de rectangulo)
                valorx = ((x2 + w2) / 2)
                valory = ((y2 + h2) / 2)
                print("x: ", valorx, "y: ", valory)

                # ultima posicion registrada de la base
                xbase = valorx
                ybase = valory
                # es detectado
                detectado = True
                # frenamos ciclo, evita analizar contornos posiblemente erroneos
                break

                print("ENCONTRO LAS COORDENADAS")

        # hacer funcionar los motores respecto a la posicion x,y para centrar color
        if (detectado):
            if (xbase != 0 and ybase != 0):
                # si no cumple con coordenadas, movemos motores para ajustarlas
                if (enX == False):
                    enX = verificarCoordenadasX2(xbase)
                else:
                    if (xbase > maximoX):
                        enX = False
                    if (xbase < minimoX):
                        enX = False
                if (enY == False):
                    enY = verificarCoordenadasY2(ybase)
                else:
                    if (ybase < minimoY):
                        enY = False
                    if (ybase > maximoY):
                        enY = False

        # si cumple con coordenadas, almacenamos en la base
        if (enX and enY):
            # se cumple la distancia aproximada entre base y carro
            almacenado = True

        # frenamos la funcion
        if (almacenado): break

        # mostrar por pantalla los frames y filtros (solo en el pc)
        # cv2.imshow("markbase", markbase)
        # cv2.imshow("fram2", fram2,)

        # tecla escape para frenar funcion (solo en pc)
        k = cv2.waitKey(5) & 0xff
        if k == 27:
            break


def encontrarBola():
    print("INICIANDO BUSQUEDA BOLA")

    # variables definidas anteriormente como limites
    global minimoXbola
    global maximoXbola

    global minimoYbola
    global maximoYbola

    # almacena ultima posicion como auxiliares
    xbola = 0
    ybola = 0
    # booleano auxiliar para la deteccion
    detectado = False

    # verifican si cumple con las coordenadas
    enX = False
    enY = False

    # permite detener la funcion cuando termina de capturar
    captura = False
    while True:
        # lectura de video
        ret, fram = cap.read()
        # dimensiones que tiene el frame
        h, w, m = fram.shape

        # convertir el frame de BGR a HSV
        hsv = cv2.cvtColor(fram, cv2.COLOR_BGR2HSV)
        # busca los rangos de colores que se encuentran en el frame
        mark1 = cv2.inRange(hsv, low_amarillo, high_amarillo)
        mark2 = cv2.inRange(hsv, low_amarillo2, high_amarillo2)
        # combinar ambos rangos de colores para una mejor deteccion
        markbola = cv2.add(mark1, mark2)

        # busca contornos que cumpla con los rangos de colores
        _, cnts, _ = cv2.findContours(markbola, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # ciclo que analiza todos los contornos que encuentra
        for c in cnts:
            # area del contorno a analizar (pixeles)
            area = cv2.contourArea(c)

            # precision de aproximacion para deteccion
            epsilon = 0.009 * cv2.arcLength(c, True)
            # verifica el tipo de contorno
            approx = cv2.approxPolyDP(c, epsilon, True)

            # si cumple con condiciones, debe ser pelotita
            if (area > 60 and len(approx) >= 15):
                # moments = cv2.moments(markbola)
                # coorX = int(moments['m10']/moments['m00'])
                # coorY = int(moments['m01']/moments['m00'])

                # calcula coordenadas y tamaño del contorno
                xx, yy, ww, hh = cv2.boundingRect(approx)  # c
                # momentos del circulo
                M = cv2.moments(approx)  # c

                # determinar punto central del circulo
                if (M["m00"] == 0): M["m00"] = 1
                x = int(M["m10"] / M["m00"])
                y = int(M['m01'] / M['m00'])
                # radio del circulo
                radio = x - xx
                # dibujar circulo en frame
                cv2.circle(fram, (x, y), radio, (0, 255, 255), 3)

                print("X: ", x, " Y: ", y, )
                print(radio)

                # ultima posicion registrada, para la pelota detectada
                xbola = x
                ybola = y
                # es detectada
                detectado = True
                # frenamos ciclo, evita analizar contornos posiblemente erroneos
                break

                # print(area)
                print("ENCONTRO LAS COORDENADAS")

        # hacer funcionar los motores respecto a la posicion x,y
        if (detectado):
            if (xbola != 0 and ybola != 0):
                # si no cumple con coordenadas, movemos motores para ajustarlas
                if (enX == False):
                    enX = verificarCoordenadasX2(xbola)
                else:
                    if (xbola > maximoXbola):
                        enX = False
                    if (xbola < minimoXbola):
                        enX = False
                if (enY == False):
                    enY = verificarCoordenadasY2(ybola)
                else:
                    if (ybola < minimoYbola):
                        enY = False
                    if (ybola > maximoYbola):
                        enY = False

        # si cumple con coordenadas, almacenamos en la base
        if (enX and enY):
            if (radio >= 30 and radio <= 50):
                # se cumple la distancia aproximada que debe haber entre bola y carro
                captura = True

        # frenamos la funcion
        if (captura): break

        # mostrar por pantalla los frames y filtros (solo en el pc)
        # cv2.imshow("markbola",markbola)
        # cv2.imshow("fram", fram,)

        # tecla escape para frenar funcion (solo en pc)
        k = cv2.waitKey(5) & 0xff
        if k == 27:
            break


def main():
    # buscar bolas
    # detectar bola, ir en la direccion
    # se acerca y procede a capturar
    # buscar otra bola
    # buscar la base
    # acercar, soltar y retrocede

    encontrarBola()
    Parar()
    sleep(0.05)

    AtraparPelota()

    buscarObjetivo()
    Parar()
    sleep(0.05)

    encontrarBola()
    Parar()
    sleep(0.05)

    AtraparPelota()

    buscarObjetivo2()
    Parar()
    sleep(0.05)

    """encontrarBase()
    Parar()
    sleep(0.05)"""

    for i in range(5):
        Adelante()
        sleep(0.5)
        Parar()
        sleep(0.05)

    SoltarPelotas()
    Parar()


main()

# termina video camara
cap.release()

# destruir ventanas de frames (solo en pc)
# cv2.destroyAllWindows()


""" 
# reemplazamos la funcion pulseIn de arduino (no funciona en ArduinoApi), usamos time de python
# google pulsein arduino (foro arduino)
def funcionPulseIn():
    #
    tiempo1 = 0.0
    tiempo2 = 0.0
    tiempo = 0.0
    estadopin = a.digitalRead(OUT)
    if (estadopin == a.HIGH):
        tiempo1 = time.time()
        while(estadopin == a.HIGH):
            #tiempo2 = time.time()
            estadopin = a.digitalRead(OUT)
        tiempo2 = time.time()
        tiempo = round(tiempo2 - tiempo1, 5)
        #print(tiempo)
        return tiempo
    else:
        tiempo1 = time.time()
        while(estadopin == a.LOW):
            #tiempo2 = time.time()
            estadopin = a.digitalRead(OUT)
        tiempo2 = time.time()
        tiempo = round(tiempo2- tiempo1, 5)
        #print(tiempo)
        return tiempo

def Color():
    a.digitalWrite(S2, a.LOW)
    a.digitalWrite(S3, a.LOW)
    RED = funcionPulseIn()

    a.digitalWrite(S3, a.HIGH)
    BLUE = funcionPulseIn()

    a.digitalWrite(S2, a.HIGH)
    GREEN = funcionPulseIn()"""

"""while True:

    #Color()
    print("R intesity:")
    print(RED) #, a.DEC)   #PUEDE QUE NO FUNCIONE EL DEC POR SER DEL SERIAL, BUSCAR IMPORTAR SERIAL
    print(" G intesity:")
    print(GREEN) #, a.DEC)
    print(" B intesity:")
    print(BLUE) #, a.DEC)"""