from_keyboard_node: usar TF para sistema de coordenadas global?

AS5048: Disminuir incertidumbre en las mediciones?

PID: crear ejemplo básico, en Raspberry o PC un action client-server. Debe recibir el tópico de los AS5048 y publicar en los motores (velocidad?). Mastering ROS p.46.

DIRECCION DEL MOTOR, EL float NO MANEJA VALORES NEGATIVOS!!!

TODO: Check if it manages OK the different incoming messages (from left and right encoder) 

max_speed y max_turn EN ACTION CLIENT. MODIFICAR EN BASE A LOS CAMBIOS DESDE EL TECLADO!!

-------------------------------------
CAMBIAR INSTRUCTIVO PARA LA GENERACION DE MENSAGES PERSONALIZADOS EN ARDUINO:

MI IMPLEMENTACION:

EN catkin_ws/src/../ 

roscore

rosrun rosserial_arduino make_libraries.py libraries/

SE GENERO UNA CARPETA libraries DONDE ESTA LA CARPETA robot_msgs. SE COPIA AL DIRECTORIO DE ARDUINO LIBRARIES.