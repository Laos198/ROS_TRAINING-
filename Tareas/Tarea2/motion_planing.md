# Navegación 
Desarrollo 

## 

| Código | Description |
| ------:| ----------- |
| ***Asignatura*** | Código del Trabajo o Número de Tarea | 
| **TSM-2023-1** |  T3 |

## Contenido
- [Navegación](#navegación)
	- [](#)
	- [Contenido](#contenido)
	- [Problema](#problema)
	- [Desarrollo](#desarrollo)
		- [Algoritmo](#algoritmo)
		- [Codigo propuesto](#codigo-propuesto)
	- [Conclusiones](#conclusiones)
	- [Autor](#autor)
	- [Referencias](#referencias)

## Problema
Un robot móvil del tipo (2,0) se encuentra en el punto P(x0, y0) con una rotación de θ radianes en su eje Z. Se requiere llegar al punto P(x1, y1) sin importar su orientación (θ radianes de giro sobre su eje Z). 


## Desarrollo
Cuando hablamos de Navegación en robótica es que el móvil tiene que llegar a un punto particular sin colisionar con algún objeto en su camino. El robot tiene que estar al tanto de los obstáculos y debe moverse libremente. 


### Algoritmo 


En este algoritmo se propone que al momento de iniciar el proceso, el punto donde se encuentre el robot sea definido por el sistema el cual ya debería previamente estbalecido. Para obtener la orientación del robot se utiliza las coordenadas que ya estan en el sistema mediante las siguientes lineas de código : 
``x = msg.pose.pose.position.x`` 
``y = msg.pose.pose.position.y``

Pero theta se representa mediante cuaterniones, y thetha representa el giro en el eje z, debemos convertirlo a angulos eulerianos, usando una instrucción que haga la transformación entre sí[[4]](#4): 

 ``rot_q = msg.pose.pose.orientation``
``( _, _, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])``

Así obteniendo las tres variables que definen la posición actual del móvil. 

Lo que sigue del algoritmo es evaluar su posición que el objetivo lo que hace unas simples evaluaciones de diferencias entre sí para variar su velocidad angular o lineal dependiendo de los valores obtenidos. 
![Diagrama de Flujo (2)](https://user-images.githubusercontent.com/20031100/200921680-22b08941-6ea8-4708-94a4-769c3f47451e.png)

La relevancia de usar la función **atan2(param_y, param_x)** es que el resultado de efectuarla te dice en en qué cuadrante se sitúa la coordenada que buscamos. Ese resultado da valores de  PI a - PI [[1]](#1)

Y los errores que podría ocasionar o bien, los errores acumulados del algoritmo podrían generarse por la resolución a la que estarían ajustados las diferencias. Ahora, si hablamos de errores en el mundo "real" podrían ocasionarse ya sea por una superficie que no es totalmente lisa o que haya alguna pequeña modificación en las llantas. [[2]](#2)

### Codigo propuesto
Este código propone una posición objetivo y recaba una posición inicial del estado del robot en la simulación de gazebo. Hasta que la pose en su orientación del robot sea la misma en la triangulación de las diferencias de puntos, avanzará, mientras no sea así, este seguirá girando.
dejo el link del script a continuación: 

https://github.com/Laos198/ROS_TRAINING-/blob/e4222c427bf0f3deb316b240ca2e77d0a4c1a76c/Tareas/Tarea2/motion_planin2.py

<pre><code>
 vel = Twist()   
 # se obtiene la posicion de inicio del móvil
 xm = msg.pose.pose.position.x
 ym = msg.pose.pose.position.y
 rot_q = msg.pose.pose.orientation
 ( _, _, thetam) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
 
 #coordenadas objetivo
 obj = Point ()
 obj.x = 5
 obj.y = 5
 
 
#Mientras  la posisicón actual del móvil no sean las mismas que las coordenadas objetivo entonces 
while (obj.x != xm) and (obj.x != ym): 
    vec_x = obj.x - xm
    vec_y = obj.y - ym
    angulo = atan2(vec_y,vec_x)

    if abs(angulo - thetam) > 0.1: 
	vel.linear.x = 0.0
	vel.angular.z = 0.5

    else: 
	vel.linear.x = 0.5
	vel.angular.z = 0.0

</code></pre>

Yo creo que si la posición final requiriera una orientación específica, no cambiaría en esencia el código, simplemente tendría que repetir la misma acción del principio que sería recabar la información y empatar las las orientaciones. 

## Conclusiones
Este tipo de navegación podría ser considerado el más sencillo, así mismo, existen varios otros que ocupan diferentes formas para la obtención de datos y por ende diversas formas de hacer la locaclización del robot. 
 

## Autor
| Iniciales  | Description |
| ----------:| ----------- |
| **DMC**  | Diego Méndez Carter [GitHub profile](https://github.com/Laos198) |

## Referencias
<a id="1">[1]</a> "Python math.atan2() Method". W3Schools Online Web Tutorials. https://www.w3schools.com/python/ref_math_atan2.asp (accedido el 9 de noviembre de 2022).

<a id="2">[2]</a>(s/f)"Odometría". Prof. Kryscia Ramírez Benavides de http://www.kramirez.net/Robotica/Material/Presentaciones/Odometria.pdf (accedido el 9 de noviembre de 2022).

<a id="3">[3]</a> https://www.iteramos.com/pregunta/42974/cual-es-la-diferencia-entre-atan-y-atan2-en-c


<a id="4">[4]</a> "[ROS Q&A] 053 - How to Move a Robot to a Certain Point Using Twist - The Construct". The Construct. https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/ (accedido el 10 de noviembre de 2022).

