# **Página Web Bitácora**

Soy Sebastián Sánchez Escobar, estudió ingeniería Mecatrónica en la Ibero Puebla, voy en primer semestre, me gustaría aprender más sobre la automatización e industria 4.0. Me gusta escuchar música, entrenar y salir con mis amigos. 

Me siento orgulloso de la disciplina que he construido y he desmostrado en varias ocasiones, gracias a este esfuerzo he desarrollado y mantenido buenos hábitos que mejoran mi vida.

Mi correo es: 196866@iberopuebla.mx 

<img src="recursos/imgs/fotoperfil.png" width="200">

# **Trabajos/Proyectos**
### **Controlar luz LED por Bluetooth.**
<video controls>
  <source src="recursos/imgs/ControlarLEDBluetooth.mp4" type="video/mp4">
</video>
En este trabajo controlamos el encendido y apagado de una luz LED con un teléfono mediante Bluetooth.
Ocupamos: Protoboard, ESP 32, LED.


### **Controlar luz LED con boton.**
<video controls style="width: 50%; max-width: 300px;">
  <source src="recursos/imgs/PXL_20250912_163927244.TS(1).mp4" type="video/mp4">
</video>
En este trabajo controlamos el encendido y apagado de una luz LED con un botón.
Ocupamos: Protoboard, ESP 32, LED, botón.

### **Cambio de giro de motor**
<video controls style="width: 75%; max-width: 300px;">
  <source src="recursos/imgs/cambio-giro-motor.mp4" type="video/mp4">
</video>
En este trabajo controlamos la dirección de giro de un motor.
Ocupamos: Protoboarb, ESP 32, Puente H, Motorreductor TT.

### **Cambio gradual de velocidad de motor**
<video controls style="width: 75%; max-width: 300px;">
  <source src="recursos/imgs/cambio-gradual-giro-motor.mp4" type="video/mp4">
</video>
Aqui el mismo circuito que en el trabajo pasado, pero cambiamos el código del ESP32 para que gradualmente cambie la velocidad.

### **Servomotor ciclo 0°-10°-0°-20°-0°-30°... hasta 180°**
<video controls style="width: 75%; max-width: 800px;">
  <source src="recursos/imgs/servomotor-ciclo-0-10-0-20.mp4" type="video/mp4">
</video>
En este trabajo hicimos que un servomotor fuera aumentando de 10° en 10° pero regresando a 0° entre cada subida.
Ocupamos: Protoboarb, ESP 32, servomotor.

## **PROYECTO: Coche a control remoto Bluetooth**
<img src="recursos/imgs/coche codigo.jpg" width="400">     

En este proyecto construimos un coche a control remoto Bluetooth. El material que usamos para el coche:    
- 4 Motorreductores TT    
- 4 Llantas Omnidireccionales    
- 2 Puentes H  
- 1 Protoboard   
- 1 ESP 32  
- Jumpers  
- 2 Pilas  
- Control Xbox  
Las 2 Pilas se conectan para alimentar todo el sistema. Los Puentes H se usan para poder controlar los 4 motores TT (1 Puente H por 2 motores TT), cada Puente H controla la dirección y velocidad de dos motores a la vez.   
El ESP32 se pone en la Protoboard y se conecta a las entradas de los Puentes H usando Jumpers.    
El código dentro del ESP32 está configurado para conectarse al control Xbox, luego lee el movimiento del joystick y dependiendo de la dirección del joystick se activa una función para que los motores se activen de manera específica para moverse en esa dirección.  
<img src="recursos/imgs/pertesfinalescoche.jpg" width="200">   


### **Código Python Vision por Cámara, dibujar en cámara**

En este trabajo logramos iniciar la cámara en python, modificar los colores y dibujar líneas y circulos.
```
import cv2
import numpy as np

video = cv2.VideoCapture(0)

#centrox=0
#centroy=0

while True:

    ret, img = video.read()

    #img2 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #img3 = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    #img4 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if not ret:
        break


    cv2.imshow("Mivideo",img)



    #DIBUJAR COSAS
    #draw = img.copy ()
    #LINEA
    #cv2.line(draw,(0,0),(100,100),(0,0,255),thickness=3, lineType=cv2.LINE_AA)
    #CIRCULO
    #cv2.circle(draw,(centrox,centroy),50,(0,0,255),thickness=3, lineType=cv2.LINE_AA)
    #cv2.imshow("Mivideodraw",draw)
    #centrox=centrox+1
    #if centrox<300:
    #    centrox=centrox+1
    #else:
    #    centrox=0


    #CAMBIOS DE COLORES
    #imgCopia = img.copy()
    #imgCopia[:, :, 1] = 0
    #imgCopia[:, :, 0] = 0
    #imgCopia[0:300, 0:300, 2] = 0
    
    #MOSTRAR IMAGEN
    #cv2.imshow("Mivideo1",img2)
    #cv2.imshow("Mivideo2",img3)
    #cv2.imshow("Mivideo3",img4)
    #cv2.imshow("Mivideo",no_blue)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()

```


### **Código Python Vision por cámara. Rastreo de cara**

Este código Python utiliza OpenCV para realizar detección de rostros en tiempo real. Esto lo logra cargando un modelo pre-entrenado de detección de rostros basado en Caffe. Si encuentra un rostro con una confianza superior al umbral (detection_treshold), dibuja un cuadrado de color verde y muestra el valor de la confianza que cree que sea un rostro alrededor del rostro.

```
import cv2
import numpy as np

video = cv2.VideoCapture(0)

#variables de detección

mean = [104,117,123]
scale = 1.0
in_width = 300
in_height = 300

detection_treshold = 0.5

net = cv2.dnn.readNetFromCaffe('models/deploy.prototxt', 'models/res10_300x300_ssd_iter_140000.caffemodel')

def detect(frame, net, scale, in_width, in_height):
    h = frame.shape[0]
    w = frame.shape[1]
    blob = cv2.dnn.blobFromImage(frame, scalefactor=scale,
                                 size=(in_width, in_height), mean=mean, swapRB=False, crop=False)
    # Pasar el blob a la red
    net.setInput(blob)
    # Pasra el blob a la red
    detections = net.forward()
 
    # Procesar las detecciones
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > detection_treshold:
           
            #Extraer las coordenadas del bounding box de la detección
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (x1, y1, x2, y2) = box.astype('int')
           
            # Dibujar el bounding box y el texto
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = 'Confidence: %.4f' % confidence
            label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - label_size[1]), (x1 + label_size[0], y1 + base_line),
                          (255, 255, 255), cv2.FILLED)
            cv2.putText(frame, label, (x1, y1),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))
    return frame

#centrox=0
#centroy=0

while True:

    ret, img = video.read()

    #img2 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #img3 = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    #img4 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if not ret:
        break

    img2 = detect(img,net, scale, in_width, in_height)
    cv2.imshow("Mivideo",img2)
    
    #draw = img.copy ()
    #LINEA
    #cv2.line(draw,(0,0),(100,100),(0,0,255),thickness=3, lineType=cv2.LINE_AA)
    #CIRCULO
    #cv2.circle(draw,(centrox,centroy),50,(0,0,255),thickness=3, lineType=cv2.LINE_AA)
    #cv2.imshow("Mivideodraw",draw)
    #centrox=centrox+1
    #if centrox<300:
    #    centrox=centrox+1
    #else:
    #    centrox=0


    #CAMBIOS DE COLORES
    #no_blue = img.copy()
    #no_blue[:, :, 1] = 0
    #no_blue[:, :, 0] = 0
    #no_blue[0:300, 0:300, 2] = 0
    
    #MOSTRAR IMAGEN
    #cv2.imshow("Mivideo1",img2)
    #cv2.imshow("Mivideo2",img3)
    #cv2.imshow("Mivideo3",img4)
    #cv2.imshow("Mivideo",no_blue)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
```
### **Código Python Vision por cámara. Dibujar circulo alrededor de pelota.**

Este código busca el controno mayor redondo que sea de color verde para marcarlo con un circulo amarillo de seguimiento.

```
import cv2
import numpy as np

cap = cv2.VideoCapture(0)  

while True:
    ok, img = cap.read()
    if not ok:
        break

    #mostrar camara normal
    cv2.imshow("Frame", img)

    #cambio a hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #hacer mascara, solo mostrar un color
    low  = np.array([35,  60,  60], dtype=np.uint8) # low es
    high = np.array([85, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, low, high)

    #mascara final
    seg = cv2.bitwise_and(img, img, mask=mask)

    #hacer contorno
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = img.copy()


    area_mayor = 0
    #checar area mayor
    for actual in contours:
        area = cv2.contourArea(actual)
        if area > area_mayor:
            area_mayor=area
            contorno_mayor=actual
        else: 
            continue

    cv2.drawContours(seg, contorno_mayor, -1, (0,255,0), 2)

    (x, y), radius = cv2.minEnclosingCircle(contorno_mayor)
    cv2.circle(out, (int(x), int(y)), int(radius), (0,255,255), 2)
    cv2.circle(out, (int(x), int(y)), 2, (0,0,255), 2)

    #cv2.imshow("Mask", mask) #mascara
    #cv2.imshow("Segmentado", seg)  #mascara final con color
    #cv2.imshow("hsv", hsv) 

    cv2.imshow("contornos", out) 

    if cv2.waitKey(1) & 0xFF == ord('x'):
        break

cap.release()
cv2.destroyAllWindows()
```



