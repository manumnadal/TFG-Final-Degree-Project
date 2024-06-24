.. _zigbee_recibo_bmi160:

TFG: Receive data BMI160
##################

.. contents::
   :local:
   :depth: 2

English
------
This project is responsible for receiving the accelerometer data sent by the sending device to the custom cluster that has been created. It is made up of four files: "bmi160.c" (prepares the sensor for use), "lcd1602a.c" (library for the screen), "main.c" (main code that executes) and "zb_zcl_custom_cluster.c" (custom cluster definition).

The connections of the LCD screen with the development board are shown below:

* GND -> GND 
* VCC -> 5V
* SDA -> P0.05
* SCL -> P0.04

To do this, it receives the data through a handler that will decrypt the received message and be able to obtain, in a simple way, the information it contains. With this information, the content of the cluster is changed with the data received and it will be sent through a data queue to a task in charge of displaying the data received on an LCD screen.

In order to manage the screen without complications, a library has been developed whose code can be found in the "src" folder with the name "lcd1602.c".

Español
------
Este proyecto se encarga de recibir los datos del acelerómetro enviados por el dispositivo emisor al cluster personalizado que se ha creado. Esta formado por cuatro ficheros: "bmi160.c" (prepara el sensor para su utilización), "lcd1602a.c" (librería para la pantalla), "main.c" (código principal que ejecuta) y "zb_zcl_custom_cluster.c" (definición del cluster personalizado).

Las conexiones de la pantalla LCD con la placa de desarrollo se muestra a continuación:

* GND -> GND 
* VCC -> 5V
* SDA -> P0.05
* SCL -> P0.04

Para ello, recibe los datos a través de un manejador que descifrará el mensaje recibido pudiendo obtener, de manera simple, la información que contiene. Con esta información, se cambia el contenido del cluster con los datos recibido y se envíara a través de una cola de datos a una tarea encargada de mostrar los datos recibidos por una pantalla LCD.

Para poder manejar la pantalla sin complicación, se ha desarrollado una librería cuyo código se puede encontrar en la carpeta "src" con el nombre de "lcd1602.c".

