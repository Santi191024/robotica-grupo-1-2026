Descripción

Este proyecto consiste en el diseño de una línea de ensamble automatizada compuesta por tres robots manipuladores encargados del ensamblaje y un robot de transporte encargado de movilizar la base del producto entre las diferentes estaciones de trabajo. El proceso tiene como objetivo ensamblar una caja de dimensiones 25 cm × 15 cm × 1.5 cm sobre la cual se deben ubicar tres pelotas de ping-pong en posiciones previamente definidas. Cada estación incorpora un robot manipulador responsable de tomar una pieza desde un repositorio cercano y colocarla en la zona correspondiente de la base, mientras que el robot de transporte garantiza el desplazamiento secuencial del producto a lo largo de la línea de producción.

La solución debe desarrollarse dentro de un área máxima de 1.25 m × 1.25 m y cumplir las restricciones establecidas en la guía, incluyendo el uso de manipuladores de al menos tres grados de libertad con configuraciones RRR o SCARA, motores DC con reductor y sensores de posición tipo encoder. Debido a que no se permite el uso de sistemas de visión artificial, todas las posiciones de recogida, transporte y ensamblaje deben ser previamente conocidas y modeladas mediante herramientas de cinemática y planeación de trayectorias.

Para garantizar el correcto funcionamiento del sistema, se implementará una arquitectura basada en ROS2 que permita coordinar las acciones de todos los robots, gestionar la secuencia de ensamblaje y supervisar el estado de cada estación. Adicionalmente, se desarrollarán los modelos cinemáticos, estrategias de control y simulaciones en Gazebo y RViz con el fin de validar el desempeño de la línea de ensamble antes de una posible implementación física. De esta manera, el proyecto integra conceptos de robótica, automatización y control para resolver una tarea de ensamblaje colaborativo similar a las utilizadas en entornos industriales reales.

Integrantes:

Oscar ayala 
,Vanessa Cardenas
,Manuel Hoyos


Estado del proyecto:

70%

Tecnologias Usadas:

ESP32
Arduino IDE
ROS2
Rviz
Gazebo
SolidWorks
