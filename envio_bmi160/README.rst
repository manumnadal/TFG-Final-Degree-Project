.. _zigbee_envio_bmi160:

Zigbee TAEE: Send data BMI160
####################

.. contents::
   :local:
   :depth: 2

English
------

This project is responsible for collecting data from the BMI160 accelerometer and sending it to the data reception platform.

The connection of the accelerometer pins with the board is shown below:

* VIN -> 5V
* GND -> GND
* SCL -> P0.04
* SDA -> P0.05
* INT1 -> P0.07
* INT2 -> P0.06

The collection is carried out through a task, which obtains the data through the I2C protocol incorporated into the sensor board. The data obtained will be the acceleration in the three axes of space (AX, AY, AZ), in addition, a float casting is performed due to the inconvenience of the format provided by I2C (the data is sent, through the protocol, using two variables: one containing the integer part and another with the decimal part). When button 2 on the board is pressed, and only if a compatible device is found, the data will begin to be sent from the collection task to the sending function.

Once the data reaches the sending function, the write request function to the custom cluster is constructed with the three values ​​it receives. Once finished, it will send another message at the frequency that the sensor measures acceleration.

Español
------
Este proyecto se encarga de recolectar datos del acelerómetro BMI160 y enviarlos a la plataforma de recepción de datos.

La conexión de los pines del acelerómetro con la placa se muestra a continuación:

* VIN -> 5V
* GND -> GND
* SCL -> P0.04
* SDA -> P0.05
* INT1 -> P0.07
* INT2 -> P0.06

La recolección se realiza a través de una tarea, la cual obtiene los datos a través del protocolo I2C incorporado en la placa del sensor. Los datos que obtiene serán la aceleración en los tres ejes del espacio (AX, AY, AZ), además se realiza un casting a float debido a la incomodidad del formato que proporciona I2C (los datos son enviados, a través del protocolo, usando dos variables: una que contiene la parte entera y otra con la parte decimal. Lo cual es ineficiente). Cuando se pulse el botón 2 de la placa, y solo si se encuentre un dispositivo compatible, se empezará a enviar los datos desde la tarea de de recolección a la función de envío.

Una vez los datos llegan a la función de envío, se construye la función de petición de escritura al cluster personalizado con los tres valores que recibe. Una vez finalizado, enviará otro mensaje a la frecuencia que el sensor mida la aceleración.