[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/yfUBpM1Z)
# P7-2024-ChaseAndRunaway

## Graficos
![FSM](https://github.com/Docencia-fmrico/p7-chase-and-runaway-jmartinm2021/assets/92941332/2bf7ae0f-c4cc-4fa1-916c-2929bb525d36)
![Arquitectura](https://github.com/Docencia-fmrico/p7-chase-and-runaway-jmartinm2021/assets/92941332/37ab9289-cc56-4efe-84b2-00e9fd7ef2db)

## Enubnciado
Implementa en ROS 2, siguiendo una arquitectura *subsumption*, un comportamiento para un robot que cambie entre 2 roles, el de *policía* y el de *ladrón*. El robot comienza siendo *policía*; este rol consiste en:

1. buscar a una persona en el entorno y, cuando la encuentra,
2. perseguirla

Para ser *policía*, se deberá de activar un comportamiento similar al implementado en la práctica anterior (producción de TFs a partir de `vision_msgs/msg/Detection3DArray` y publicación de velocidades lineales y angulares pasando por un PID). Una vez que el robot está a 1 metro (o cualquier otro valor de distancia definido por un parámetro) o menos de la persona, girará sobre sí mismo `PI` radianes y cambiará su rol, activando el comportamiento de *ladrón*. Este comportamiento consiste en un *bump and go* tal y como el visto en clase. Transcurridos 10 segundos (o cualquier otro valor de tiempo definido por un parámetro), el robot volverá a su rol de *policía*. Y así cíclicamente.

Puedes emitir sonidos cada vez que el robot cambie de rol.

---

Para implementar los comportamientos necesitarás hacer uso de `cascade lifecycle nodes`. Una vez conseguido el funcionamiento solicitado, puedes hacer los cambios pertinentes para la activación en cascada del flujo de percepción (por ejemplo, activando el nodo que produce TFs en cascada cuando se activa el comportamiento de *policía*, y desactivándolo cuando deja de ser necesario).
