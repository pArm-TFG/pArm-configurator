# μArm
_Práctica de Manipuladores con el robot μArm_

### _En este repositorio..._

+ Enunciado de la práctica.
+ Documentación sobre el robot.
+ Código fuente de la práctica.
+ Documentación de la práctica.
+ Archivos README y LICENSE.

La información relativa al enunciado de la práctica se puede encontrar en el siguiente PDF: https://github.com/UPM-Robotics/uarm/blob/master/docs/statement.pdf

## Información sobre el robot

_μArm Swift Pro_ es un robot creado por la empresa **UFACTORY** el cual cuenta con cuatro grados de libertad, aunque en la práctica solo usa 3 (debido a que el extremo, la pinza, está en una posición fija respecto al suelo).

Se encuentra disponible la siguiente información:
    
1. [Manual de usuario](https://github.com/UPM-Robotics/uarm/blob/master/docs/robot-information/uArm%20pro%20User%20Manual%20v1.1.0.pdf) - http://download.ufactory.cc/docs/en/uArm%20pro%20User%20Manual%20v1.1.0.pdf
2. [Especificaciones](https://github.com/UPM-Robotics/uarm/blob/master/docs/robot-information/uArm-Swift-Specifications-171012.pdf) - http://download.ufactory.cc/docs/en/uArm-Swift-Specifications-171012.pdf
3. [Guía del desarrollador](https://github.com/UPM-Robotics/uarm/blob/master/docs/robot-information/uArm%20Swift%20Pro_Developer%20Guide%20v1.0.6.pdf) - http://download.ufactory.cc/docs/en/uArm%20Swift%20Pro_Developer%20Guide%20v1.0.6.pdf
4. [Modelo en 3D](https://github.com/UPM-Robotics/uarm/blob/master/docs/robot-information/uArm_Swift_Pro_3D_20180620.STEP) - http://download.ufactory.cc/developer/swift/uArm_Swift_Pro_3D_20180620.STEP

El resto de la información pertinence relacionada con el brazo robótico se puede extraer de la [página web de UFACTORY](https://www.ufactory.cc/#/en/) y, en particular, de la [zona de soporte](https://www.ufactory.cc/#/en/support/technology).

## *TO-DO*

- [x] Repositorio de GitHub y organización general.
- [ ] Configuración geométrica
  - [ ] Definición de referenciales.
  - [ ] Tabla de parámetros de [Denavit–Hartenberg](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters).

- [ ] Cinemática directa del manipulador.
  - [ ] Estudio de los casos: <img src="https://latex.codecogs.com/png.latex?\vec{q}&space;=&space;(0,&space;0,&space;0),&space;\vec{q}&space;=&space;(0,&space;\pi/2,&space;0),&space;\vec{q}&space;=&space;(-\pi/2,&space;\pi/2,&space;a)&space;\vec{q}=&space;(\pi,&space;0,&space;a/2)" title="\vec{q} = (0, 0, 0), \vec{q} = (0, \pi/2, 0), \vec{q} = (-\pi/2, \pi/2, a) \vec{q}= (\pi, 0, a/2)" />
  - [ ] Función que implementa la cinemática directa del manipulador <img src="https://latex.codecogs.com/png.latex?A_0^3" title="A_0^3" />

- [ ] Cinemática inversa
  - [ ] Ecuaciones de la cinemática inversa del manipulador.
  - [ ] Función que implementa dicha cinemática - devuelve una lista con todos los posibles valores de los parámetros.

- [ ] Función Jacobiana
  - [ ] Matriz Jacobiana en el estudio de los casos <img src="https://latex.codecogs.com/png.latex?\vec{q}&space;=&space;(0,&space;0,&space;a/2),&space;\vec{q}&space;=&space;(0,&space;\pi/90,&space;0),&space;\vec{q}&space;=&space;(\pi/4,&space;-\pi/4,&space;0),&space;\vec{q}&space;=&space;(-\pi/90,&space;\pi/90,&space;0),&space;\vec{q}&space;=&space;(-\pi/4,&space;\pi/4,&space;0)&space;y&space;\vec{q}&space;=&space;(\pi/90,&space;-\pi/90,&space;a/100),&space;\vec{q}&space;=&space;(0,&space;\pi/2,&space;a/2),&space;\vec{q}&space;=&space;(\pi/90,&space;0,&space;-a/100)" title="\vec{q} = (0, 0, a/2), \vec{q} = (0, \pi/90, 0), \vec{q} = (\pi/4, -\pi/4, 0), \vec{q} = (-\pi/90, \pi/90, 0), \vec{q} = (-\pi/4, \pi/4, 0) y \vec{q} = (\pi/90, -\pi/90, a/100), \vec{q} = (0, \pi/2, a/2), \vec{q} = (\pi/90, 0, -a/100)" /> y su expresión gráfica.
  - [ ] Función que devuelve la matriz Jacobiana del manipulador dadas las coordenadas articulares.

- [ ] Singularidades y generación de trayectorias
  - [ ] Determinar las configuraciones singulares.
  - [ ] Función que devuelva una lista de coordinadas articulares las cuales, aplicadas en secuencia, permiten hacer que el manipulador describa una determinada trayectoria (función matemática).

- [ ] Representación gráfica de los resultados.

## Otros recursos

+ *[Learn LaTeX in 30 minutes](https://www.overleaf.com/learn/latex/Learn_LaTeX_in_30_minutes)*
