# Proyecto de robótica: Clasificación de frutas

Este proyecto implementa un sistema robótico que clasifica frutas según su calidad (buenas o malas) utilizando ROS (Robot Operating System), algoritmos de visión por computadora y aprendizaje automático. El sistema consta de dos nodos principales: uno que analiza la calidad de las frutas y otro que controla el robot para mover las frutas a la caja correspondiente.

---

## Requisitos del sistema

- **Linux/macOS/Windows**: Docker y VSCode son compatibles con todas estas plataformas, aunque algunas características pueden variar.
- **GPU (opcional)**: Para aprovechar la aceleración de hardware, asegúrate de tener una GPU NVIDIA compatible y los drivers instalados.

## Contenedores disponibles

Este repositorio incluye los siguientes contenedores:

1. **desktop**: Un entorno de escritorio accesible vía web (noVNC) con ROS instalado. Este contenedor funciona en cualquier SO y HW que tenga Docker instalado.
2. **local_gpu**: Un entorno local con soporte para GPU, optimizado para trabajos en el laboratorio.
3. **local**: Un entorno local sin soporte para GPU, para usar en entornos sin aceleración de hardware.

Los contenedores 2 y 3 están desarrollados para ser utilizados en los PCs del laboratorio de robótica. Sólo se pueden utilizar si se cumplen los siguientes requisitos:

- El sistema operativo host es Linux (o WSL).
- El sistema operativo host utiliza el sistema de ventanas X.
- (Versión GPU) El sistema tiene una gráfica NVIDIA con el `nvidia-container-toolkit` y sus drivers instalados.

Para el uso personal fuera del laboratorio se recomienda utilizar el contenedor **desktop**.

---

## Configuración y ejecución

### 1. Configurar el entorno

1. **Clonar el repositorio:**

   Clona el repositorio principal junto con el submódulo:

   ```bash
   git clone --recurse-submodules https://github.com/AsierBujedo/Robotica.git
   cd Robotica
   ```

2. **Construir y lanzar los contenedores:**

   Desde la raíz del proyecto debe construirse el contenedor, para eso debe ejecutarse:

   ```bash
   sudo docker-compose --profile {PROFILE} build
   ```

   y luego lanzarse:

   ```bash
   sudo docker-compose --profile {PROFILE} up -d
   ```

   Esto construirá las imágenes Docker (si no se han construido previamente) y lanzárá los contenedores necesarios para el sistema.

3. **Ingresar al contenedor:**

   Accede al contenedor donde se encuentra configurado el entorno ROS:

   ```bash
   docker exec -it nombre_contenedor bash
   ```

   Sustituye `nombre_contenedor` por el nombre del contenedor adecuado.

4. **Inicializar el entorno dentro del contenedor:**

   Una vez dentro del contenedor, navega al directorio `ros_workspace` y ejecuta:

   ```bash
   cd ~/ros_workspace
   source setup.sh
   ```

   Este script realiza lo siguiente:

   - Automatiza las acciones que deben llevarse a cabo para preparar ROS (rosdep update, catkin build...)
   - Define los alias necesarios para el proyecto:
     - `launch_robot`: Alias para lanzar el entorno del robot.
     - `launch_sim`: Alias para lanzar el simulador.
   - Configura variables de entorno necesarias para el correcto funcionamiento.

### 2. Iniciar el nodo de análisis de frutas

El nodo de análisis de frutas evalúa si las frutas son buenas o malas utilizando algoritmos de aprendizaje automático.

1. **Configurar las variables de entorno:**

   Antes de ejecutar el nodo, especifica las siguientes variables de entorno:

   ```bash
   export ROS_MASTER_URI=http://{IP_NODO_MAESTRO}:11311
   export ROS_IP={IP_LOCAL}
   ```

   - `{IP_NODO_MAESTRO}`: Dirección IP del nodo maestro de ROS.
   - `{PUERTO}`: Puerto utilizado por el nodo maestro (por defecto, 11311).
   - `{IP_LOCAL}`: Dirección IP de la máquina local.

2. **Instalar dependencias:**

   Cambia al directorio del submódulo `Robotica-AI` e instala las dependencias necesarias:

   ```bash
   cd ros_workspace/Robotica-AI
   pip install -r requirements.txt
   ```

3. **Ejecutar el nodo de análisis:**

   Lanza el nodo ejecutando el archivo principal:

   ```bash
   python src
   ```

### 3. Controlar el robot

El nodo controlador del robot se encarga de mover las frutas a las cajas correspondientes según el análisis recibido del nodo de frutas.

1. **Configurar las variables de entorno:**

   Asegúrte de configurar las mismas variables que en el nodo de frutas:

   ```bash
   export ROS_MASTER_URI=http://{IP_NODO_MAESTRO}:11311
   export ROS_IP={IP_LOCAL}
   ```

2. **Lanzar el robot:**

   Utiliza el alias definido en `setup.sh` para lanzar el robot:

   ```bash
   launch_robot
   ```

3. **Iniciar el nodo de controlador:**

   Ejecuta el script del nodo controlador:

   ```bash
   python control_robot.py
   ```

### 4. Lanzar el simulador

Para probar el sistema en un entorno simulado, utiliza el alias `launch_sim`:

```bash
launch_sim
```

Esto abrirá el simulador y permitirá probar el sistema sin necesidad de hardware físico.

---

## Solución de problemas

1. **El nodo no puede conectarse al nodo maestro:**

   - Verifica que las variables `ROS_MASTER_URI` y `ROS_IP` están configuradas correctamente.
   - Asegúrate de que la dirección IP y el puerto del nodo maestro sean accesibles.

2. **Errores de dependencias en el nodo de frutas:**

   - Asegúrte de que todas las dependencias están instaladas ejecutando:
     ```bash
     pip install -r requirements.txt
     ```

3. **Problemas con Docker:**

   - Verifica que Docker y Docker Compose estén correctamente instalados y configurados.
   - Reinicia los contenedores con:
     ```bash
     docker-compose down && docker-compose --profile {PROFILE} up
     ```
---


## Descripción de los archivos principales

### Explicación del flujo del sistema

El sistema implementado en este proyecto sigue un flujo de trabajo claramente definido para lograr el traslado de diferentes frutas hacia una caja según su clasificación:

1. **Nodo de órdenes** (`debug.py`):
   - El usuario puede interactuar directamente con el sistema a través de un menú interactivo.
   - Este nodo permite enviar comandos individuales (como mover el robot a la posición inicial o abrir/cerrar la pinza) o ejecutar tareas completas como clasificar frutas buenas y malas.
   - Los comandos se publican en el topic `/consignas`.

2. **Control del robot** (`control_robot.py`):
   - Este nodo escucha los comandos publicados en `/consignas` y realiza las acciones necesarias utilizando MoveIt para planificar movimientos seguros del robot.
   - Adicionalmente, registra métricas de rendimiento en una base de datos InfluxDB para trazabilidad.

3. **Ejecución de comandos** (`commands.py` y `controller.py`):
   - Los comandos específicos del robot (como `COGER_FRUTA` o `CAJA_BUENA_ARRIBA`) se definen en `commands.py` y se gestionan mediante colas en `CommandQueue`.
   - `controller.py` utiliza estas colas para definir y ejecutar flujos de trabajo completos, como mover una fruta a la caja correspondiente.

4. **Gestión de métricas** (`influx.py`):
   - Este módulo registra las métricas del robot (posiciones, velocidades, esfuerzos, etc.) en InfluxDB para monitoreo y análisis posterior.

---

### 1. `commands.py`
Este archivo define los comandos utilizados para controlar el robot y su sistema de colas. Contiene:
- **Clase `Command`**: Enumera los comandos disponibles, como `COGER_FRUTA`, `POSICION_INICIAL`, entre otros.
- **Clase `CommandQueue`**: Implementa una cola para gestionar y enviar comandos al robot mediante ROS, publicando en el topic `/consignas`.

### 2. `control_robot.py`
Este archivo implementa la clase principal para controlar el robot mediante ROS y MoveIt. Funciones destacadas:
- **`handle_command`**: Escucha los comandos publicados en `/consignas` y ejecuta acciones específicas, como mover a posiciones predefinidas o manipular la pinza.
- **`add_floor` y `add_box_to_planning_scene`**: Añaden objetos a la escena de planificación para evitar colisiones.
- **`mover_pinza`**: Controla la apertura y cierre de la pinza.
- **Integración con InfluxDB**: Registra métricas del robot, como velocidades y esfuerzos, en una base de datos para trazabilidad.

### 3. `controller.py`
Define funciones de alto nivel para ejecutar tareas completas utilizando `CommandQueue`:
- **`poner_caja_buena`**: Lleva una fruta a la caja de frutas buenas.
- **`poner_caja_mala`**: Lleva una fruta a la caja de frutas malas.

### 4. `debug.py`
Un script interactivo para probar y depurar el sistema:
- Permite al usuario seleccionar comandos manualmente o ejecutar funciones como `poner_caja_buena` o `poner_caja_mala`.
- Publica comandos en `/consignas` o llama a funciones del controlador directamente.

### 5. `influx.py`
Proporciona una clase para interactuar con InfluxDB:
- **`write_data`**: Escribe datos en la base de datos InfluxDB con soporte para múltiples campos y etiquetas.
- **`close`**: Cierra la conexión con InfluxDB.

---

### Documentación del módulo Robotica-AI

Este módulo se encarga de procesar las imágenes capturadas, extraer características, realizar predicciones sobre la calidad de las frutas, y controlar las acciones del robot en consecuencia. A continuación, se detalla el funcionamiento de los archivos principales:

---

### 1. `__main__.py`
Este archivo es el punto de entrada principal del módulo. Contiene la lógica para capturar imágenes, procesarlas, predecir su clase y enviar las instrucciones al robot:
- **Carga de modelos**: Utiliza el archivo `model_predictor.py` para cargar el modelo de predicción entrenado y el codificador de etiquetas.
- **Procesamiento de imágenes**: Captura fotogramas en vivo desde la cámara y permite procesarlos al presionar la tecla `c`.
- **Predicción de clase**: Predice si la fruta es buena o defectuosa usando el modelo de aprendizaje automático cargado.
- **Control del robot**: Dependiendo de la predicción, ejecuta funciones como `poner_caja_buena` o `poner_caja_mala` del archivo `controller.py` para mover la fruta a la caja correspondiente.

---

### 2. `vision.py`
Este archivo implementa las funciones necesarias para procesar las imágenes capturadas y extraer características relevantes para la clasificación de frutas. Principales funciones:
- **`adjust_brightness_contrast`**: Ajusta el brillo y contraste de la imagen para mejorar la calidad del procesamiento.
- **`process_image`**: Aplica filtros Gaussianos y Canny para encontrar contornos en la imagen.
- **`calculate_rugosity`**: Calcula la rugosidad de la imagen usando la varianza.
- **`calculate_shape_features`**: Extrae características de forma como área, perímetro, relación de aspecto y momentos de Hu.
- **`calculate_glcm_features`**: Calcula características basadas en la textura usando GLCM (Haralick).
- **`get_color_ranges`**: Obtiene los tonos dominantes en la imagen después de aplicar máscaras de color.

---

### 3. `commands.py`
Define los comandos y la cola utilizada para comunicarse con el robot:
- **Clase `Command`**: Enumera comandos como `COGER_FRUTA`, `POSICION_INICIAL`, `ABRIR_PINZA`, etc.
- **Clase `CommandQueue`**: Permite agregar comandos a una cola y enviarlos secuencialmente al robot mediante el topic ROS `/consignas`.

---

### 4. `controller.py`
Define funciones para ejecutar flujos de trabajo completos utilizando la cola de comandos:
- **`poner_caja_buena`**: Lleva una fruta clasificada como buena a la caja correspondiente.
- **`poner_caja_mala`**: Lleva una fruta clasificada como defectuosa a la caja de descartes.

---

### 5. `model_predictor.py`
Este archivo se encarga de cargar el modelo de aprendizaje automático y realizar predicciones:
- **`load_models`**: Carga el modelo de clasificador y el codificador de etiquetas.
- **`create_feature_dataframe`**: Convierte las características extraídas de la imagen en un DataFrame compatible con el modelo.
- **`predict_class`**: Utiliza el modelo cargado para predecir la clase de la fruta y decodifica la etiqueta correspondiente.

---

