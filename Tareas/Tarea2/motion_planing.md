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
Este es una adaptación al código visto en clase para encontrar el objeto más cercano al que está situado el robot, proponiedo que este se mueva al objeto más cercano sin importar su punto de inicio.

<pre><code>
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from tf.transformations import euler_from_quaternion

global xi
global yi
global thetai

from tb3_cmd.srv import GetClosest, GetClosestResponse, GetDistance, GetDistanceResponse

class GazeboUtils(object):
    def __init__(self):
        pass

    def getWorldProperties(self):
        try:
            get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            wp = get_world_properties()
            if wp.success:
                return wp
            else:
                rospy.logwarn(f"al invocar el servicio se recibio el estatus: {wp.success}")
                return None
        except rospy.rospy.ServiceException as se:
            rospy.logerr(f"Error al llamar '/gazebo/get_world_properties': {se}")    

    def getModelState(self, model_name, relative_entity_name='world'):
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            ms = get_model_state(model_name, relative_entity_name)
            if ms.success:
                return ms
            else:    
                rospy.logwarn(f"al invocar el servicio se recibio el estatus: {ms.success}")
                return None
        except rospy.rospy.ServiceException as se:
            rospy.logerr(f"Error al llamar '/gazebo/get_model_state': {se}")    
        

class DistanceMonitor():
    def __init__(self):
        self._landmarks = {}
        self._excluded_objs = ['ground_plane', 'turtlebot3_waffle']
        self._gazebo_utils = GazeboUtils()
        self._position = Point()
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odom_callback)
        self._getClosestSrv = rospy.Service('/get_closest', GetClosest, self.get_closest_srv)
        self._getDistanceSrv = rospy.Service('/get_distance', GetDistance, self.get_distance_srv)
        self._ini()

    def _ini(self):
        wp = self._gazebo_utils.getWorldProperties()
        if wp:
            for model in wp.model_names:
                if model not in self._excluded_objs:
                    ms = self._gazebo_utils.getModelState(model)
                    position = (ms.pose.position.x, ms.pose.position.y)
                    self._landmarks.update({model: position})

    def _on_odom_callback(self, msg):
        self._position = msg.pose.pose.position    

    def get_closest_srv(self, req):
        closest_landmark = ''
        closest_distance = -1
        for model_name, (x, y) in self._landmarks.items():
            dx = x - self._position.x 
            dy = y - self._position.y
            sqr_dist = (dx * dx) + (dy * dy)
            if closest_distance == -1 or sqr_dist < closest_distance:
                closest_distance = sqr_dist
                closest_landmark = model_name

        response = GetClosestResponse()
        response.object_name = closest_landmark
        response.success = True
        response.status_message = "Todo OK"

        return response


    def get_distance_srv(self, req):
        response = GetDistanceResponse()
        if req.object_name not in self._landmarks:
            response.object_distance = 0.0
            response.success = False
            response.status_message = f"El objeto '{req.object_name}' no fue encontrado."
            return response

        x, y = self._landmarks[req.object_name]        
        dx = x - self._position.x 
        dy = y - self._position.y
        response.object_distance = math.hypot(dx, dy)
        response.success = True
        response.status_message = "Todo ok"

        return response


def test_services():
    gazebo_utils = GazeboUtils()
    wp = gazebo_utils.getWorldProperties()
    if wp:
        for model in wp.model_names:
            ms = gazebo_utils.getModelState(model)
            position = (ms.pose.position.x, ms.pose.position.y) 
            print(f"model_name: {model} (x:{position[0]}, y:{position[1]})")


def main():
    rospy.init_node('distance_monitor_server')
    monitor = DistanceMonitor()
    rospy.spin()

if __name__ == '__main__':
    main()


</code></pre>

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

