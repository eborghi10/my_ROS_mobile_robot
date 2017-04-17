from_keyboard_node: usar TF para sistema de coordenadas global?

AS5048: Disminuir incertidumbre en las mediciones?

PID: crear ejemplo básico, en Raspberry o PC un action client-server. Debe recibir el tópico de los AS5048 y publicar en los motores (velocidad?). Mastering ROS p.46.

Implementar from_keyboard_node_test para reducir sustancialmente la cantidad de variables globales. AGREGAR SOLO A _arduino_core_!!

#if defined(__AVR_ATmega32U4__)
  #define USE_USBCON
#endif