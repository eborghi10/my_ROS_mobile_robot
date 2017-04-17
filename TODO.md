from_keyboard_node: usar TF para sistema de coordenadas global?

AS5048: Disminuir incertidumbre en las mediciones?

PID: crear ejemplo básico, en Raspberry o PC un action client-server. Debe recibir el tópico de los AS5048 y publicar en los motores (velocidad?). Mastering ROS p.46.

#if defined(__AVR_ATmega32U4__)
  #define USE_USBCON
#endif

Cambiar PID.action a valores std_msgs::UInt16
Cambiar result a Bool

Modificar el ControllerServer

ControllerServer.cpp (line 131): PIDController() returns float and then round().

La acción recibe el ángulo objetivo, pero para poder controlar el motor, tiene que poder leer los angulos!!