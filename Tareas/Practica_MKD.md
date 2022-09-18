# Practica de documentación en Markdown
Desarrollo 

## 

| Código | Description |
| ------:| ----------- |
| ***Asignatura*** | Código del Trabajo o Número de Tarea | 
| **TSM-2023-1** |  T1 |

## Contenido
- [1 . Introducción](#introduccion)
- [2 . Desarrollo](#desarrollo)
- [3. Conclusiones](#conclusiones)
- [4 . Autor](#autor)
- [5 . Referencias](#referencias)

## Introducción
MarkDown fue desarrollado en 2004 por John Gruber y Aaron Swartz, Markdown es un sistema de formato abierto que simplifica el proceso de estructuración del texto. Con una codificación mínima, es fácil de usar, visualmente más limpio y también puede ser convertido a HTML de manera muy sencilla.

Este lenguaje puede ser procesado en muchos programas, desde Microsoft Word hasta Openoffice.org Writer; y sus archivos se vuelven pequeños y difícilmente se rompen.

Con él, eres capaz de crear:

* Marcaciones 
* Títulos 
* Textos 
* Links 
* Códigos

Todo de una manera limpia, legible y precisa, y más rápido que si se hiciera usando HTML. MarkDown wa inmensamente más legible y está al alcance de la gente que no es desarrollador. [[1]](#1)

En esta tarea se hará práctica cpn su implementación desarrollando temas de interés para la materia de temas selectos de ingeniería mecatrónica del semestre 2023-1

## Desarrollo
### Clasificación de los robots móviles por configuración 
Los robots con ruedas pueden ser clasificados en dos categorías: 
- Omnidireccionales 
- No Holonómicos 

Y esto depende mucho por el tipo de ruedas que utiliza, ya que los robots no honolómicos utilizan ruedas convencionales como las que tienen los autos normales: Las ruedas rotan con respecto a un eje perpendicular al plano de la rueda que contiene el centro de esta. 
![image](https://user-images.githubusercontent.com/20031100/190763168-bd0aa726-2c02-46e3-8f39-21f324e52563.png) [[2]](#2)

Los sistemas omnidireccionales utilizan tipicamente ruedas mecanum (o ruedas suecas). Estas ruedas generalmente es una llant aumentada con rodillos en su circunferencia que giran libremente permitiendo un deslizamiento hacia los lados del sistema.
![image](https://user-images.githubusercontent.com/20031100/190756597-b9519143-d56e-4b72-917e-6a5ff87f8a58.png) [[2]](#2)

### Restricciones cinemáticas y diferencias entre los tipos de sistmemas
- Para los sistemas no holonómicos sabemos que la cantidad de grados de libertad controlables es menor a la cantidad de grados totales; como los carros normales, tracciones diferenciales o configuracioens de tricíclos, esto nos dice que no todas las trayectorias son posibles.

- Para los sistemas omnidireccionales (holonómicos) sabemos que la cantidad de grados de libertad controlables es igual a la cantidad de grados de libertad totales; Lo que nos dice que todas las trayectorias en el plano son posibles. [[3]](#3)

### Conceptos de localización, ruta, odometría y planeación de ruta.
- Localización: Consiste en determinar la posición del robot en relación a un mapa dado a un entorno. [[4]](#4)
- Ruta: secuencia de posiciones en las que un robot se encuentra libre de colisión con los obstáculos del espacio de trabajo. [[5]](#5)
- Odometría: Estudio de la estimación de la posición de vehículos con ruedas durante la navegación 
- Planeación de rutas: Sea un espacio de trabajo de un robot cuya planeación de ruta es aquella por la cual este desea encontrar un camino libre de colisión dada una configuración incial y una final si existe; en caso contrario que sea capaz de determinarla. [[5]](#5)

### Caracterización de la plataforma móvil TurtleBot3:
- Modelo cinemático

Como es un robot diferencial, tiene el modelo cinemátcio siguiente

![image](https://user-images.githubusercontent.com/20031100/190882535-94d602be-c1f8-484d-a60d-3976a3b4d6c2.png)

![image](https://user-images.githubusercontent.com/20031100/190882543-b3ce3e11-d0f2-4b36-9805-c756ae547b8f.png)

![image](https://user-images.githubusercontent.com/20031100/190883055-de66617c-74fd-4875-8d10-2fd724d90433.png)


$$
\mathbb{q^°} \ =\ \begin{bmatrix}
	Φ & x^° & y^°
\end{bmatrix} \ =\ \begin{bmatrix}
-r/d2 & r/2d \\
r/2 cosΦ & r/2 cosΦ \\
r/2 isnΦ & r/2 sinΦ
\end{bmatrix} \  

$$




- Sensores y actuadores que lo integran

  Según la página de Robotis, el único sensor que usa el TurtleBot3 en su versión Burger es un     LIDAR "LDS-01" o "LDS-02" 
  
  En cuanto a actuadores, tenemos que usa dos Motores 	DYNAMIXEL (XL430-W250-T) [[6]](#6)
   ![image](https://user-images.githubusercontent.com/20031100/190827583-2d136908-5c30-4047-854f-fb7e6483c27b.png)

- Nodos y Tópicos de ROS utilizados por la plataforma Turtlebot3 y sus sensores

## Conclusiones
 
 

## Autor
| Iniciales  | Description |
| ----------:| ----------- |
| **DMC**  | Diego Méndez Carter [GitHub profile](https://github.com/Laos198) |

## Referencias
<a id="1">[1]</a> ¿De qué manera Markdown simplifica la creación de textos en la web? (s/f). Hostgator.mx. Recuperado el 14 de septiembre de 2022, de https://www.hostgator.mx/blog/markdown/


<a id="2">[2]</a> Lynch, K. M., & Park, F. C. (2017). MECHANICS, PLANNING, AND CONTROL. Northwestern.edu. http://hades.mech.northwestern.edu/images/7/7f/MR.pdf


<a id="3">[3]</a> (S/f). Femexrobotica.org. Recuperado el 16 de septiembre de 2022, de https://www.femexrobotica.org/eir2016-2017/wp-content/uploads/robots_omnidireccionales.pdf

<a id="4">[4]</a> Localización y planificación de trayectorias. (s/f). Tekniker.es. Recuperado el 16 de septiembre de 2022, de https://www.tekniker.es/es/localizacion-y-planificacion-de-trayectorias

<a id="5">[5]</a>(S/f-b). Udlap.mx. Recuperado el 16 de septiembre de 2022, de http://catarina.udlap.mx/u_dl_a/tales/documentos/lis/munoz_r_o/capitulo4.pdf

<a id="6">[6]</a> ROBOTIS e-manual. (s/f). ROBOTIS E-Manual. Recuperado el 16 de septiembre de 2022, de https://emanual.robotis.com/docs/en/platform/turtlebot3/features/


<a id="7">[7]</a> "How to install python3-smbus ubuntu package on ubuntu 20.04/ubuntu 18.04/ubuntu 19.04/ubuntu 16.04". Modern Server and App Hosting Control Panel. https://zoomadmin.com/HowToInstall/UbuntuPackage/python3-smbus (accedido el 13 de mayo de 2022).

<a id="8">[8]</a> "Arm_lib". PyPI. https://pypi.org/project/Arm_lib/ (accedido el 13 de mayo de 2022).
 

