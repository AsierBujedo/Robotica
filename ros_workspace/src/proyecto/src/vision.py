import cv2
import numpy as np
import math
import mahotas as mh  # Asegúrate de tener instalada la librería mahotas
import joblib
import pandas as pd
from controller import *

# Ajustar brillo y contraste
def adjust_brightness_contrast(image, alpha=1, beta=70):
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

# Procesar imagen (adaptado a tiempo real, usando frame directamente)
def process_image(image):
    bright_image = adjust_brightness_contrast(image)
    # Preprocesamiento
    blurred = cv2.GaussianBlur(bright_image, (5, 5), 0)
    edges = cv2.Canny(blurred, 100, 200)

    # Encontrar contornos
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Crear una copia para dibujar contornos
    output = image.copy()
    cv2.drawContours(output, contours, -1, (0, 255, 0), 2)  # Contornos en verde

    return bright_image, blurred, edges, output, contours

# Calcular rugosidad (simple)
def calculate_rugosity(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    rugosity = np.var(gray)  # Varianza de la imagen
    return rugosity

# Calcular características de forma
def calculate_shape_features(contours):
    if len(contours) == 0:
        return None

    # Contorno con área máxima
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    perimeter = cv2.arcLength(largest_contour, True)
    
    # Dimensiones del bounding box
    x, y, w, h = cv2.boundingRect(largest_contour)
    aspect_ratio = float(w) / h if h != 0 else 0
    circularity = (4 * math.pi * area) / (perimeter ** 2) if perimeter != 0 else 0
    
    # Momentos Hu
    moments = cv2.moments(largest_contour)
    hu_moments = cv2.HuMoments(moments).flatten()

    # Coordenadas normalizadas del contorno
    normalized_contour = largest_contour - [x, y]
    normalized_contour = normalized_contour / [w, h]  # División elemento a elemento

    # Calcular estadísticas de las coordenadas normalizadas
    mean_coord = np.mean(normalized_contour, axis=0).flatten()
    std_coord = np.std(normalized_contour, axis=0).flatten()

    # Contar el número de contornos
    number_of_contours = len(contours)

    shape_features = {
        'number_of_contours': number_of_contours,
        'area': area,
        'perimeter': perimeter,
        'aspect_ratio': aspect_ratio,
        'height': h,
        'width': w,
        'circularity': circularity,
        'hu_moments': hu_moments,
        'mean_normalized_contour_x': mean_coord[0],
        'mean_normalized_contour_y': mean_coord[1],
        'std_normalized_contour_x': std_coord[0],
        'std_normalized_contour_y': std_coord[1]
    }

    return shape_features

# Calcular características GLCM
def calculate_glcm_features(image):
    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Calcular las características GLCM usando mahotas
    haralick_features = mh.features.texture.haralick(gray, distance=1).mean(axis=0)
    
    # Extraer las características necesarias
    contrast = haralick_features[1]       # Índice 1 es "contrast"
    correlation = haralick_features[2]    # Índice 2 es "correlation"
    energy = haralick_features[8]         # Índice 8 es "energy"
    homogeneity = haralick_features[4]    # Índice 4 es "homogeneity"
    
    return contrast, correlation, energy, homogeneity

# Obtener los rangos de los cinco colores más predominantes
def get_color_ranges(image, contours):
    if len(contours) == 0:
        return None  # No hay contornos detectados

    # Crear una máscara vacía
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    
    # Procesar cada contorno
    for cnt in contours:
        hull = cv2.convexHull(cnt)
        cv2.drawContours(mask, [hull], -1, (255), thickness=cv2.FILLED)  # Contornos llenos
    
    # Aplicar máscara y convertir a HSV
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    hsv_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
    
    # Filtrar el color blanco y tonos claros
    lower_bound = np.array([0, 50, 0])   # Excluye saturación baja y tonos blancos
    upper_bound = np.array([180, 255, 230])  # Excluye luminosidad alta (blanco)
    filtered_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    
    # Excluir el color azul
    blue_lower_bound = np.array([100, 50, 0])
    blue_upper_bound = np.array([140, 255, 255])
    blue_mask = cv2.inRange(hsv_image, blue_lower_bound, blue_upper_bound)
    filtered_mask = cv2.bitwise_and(filtered_mask, cv2.bitwise_not(blue_mask))
    
    # Aplicar la máscara filtrada
    filtered_hsv_image = cv2.bitwise_and(hsv_image, hsv_image, mask=filtered_mask)
    
    # Histograma del canal H (tonalidad)
    h_channel = filtered_hsv_image[:, :, 0]
    hist = cv2.calcHist([h_channel], [0], filtered_mask, [180], [0, 180])
    
    # Obtener los tonos dominantes
    dominant_hues = np.argsort(hist.flatten())[-5:]  # 5 tonos más comunes
    ranges = [(int(h), int(h + 10)) for h in dominant_hues]  # Rangos de color
    
    return ranges
def mover_fruta_robot(clasificacion):
    if "defecto" in clasificacion:
        print(f"{clasificacion}: Moviendo a la Caja Mala.")
        poner_caja_mala()  # Método para mover la fruta defectuosa
    else:
        print(f"{clasificacion}: Moviendo a la Caja Buena.")
        poner_caja_buena()  # Método para mover la fruta buena

# Cargar el modelo de ML previamente entrenado
clf_loaded = joblib.load('random_forest_model.pkl')  # Reemplaza con la ruta real
le = joblib.load('label_encoder.pkl')    # Reemplaza con la ruta real

# Abrir la cámara conectada por USB
cap = cv2.VideoCapture(0)  # Ajusta el índice si tienes varias cámaras conectadas

if not cap.isOpened():
    print("No se pudo abrir la cámara USB.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("No se pudo capturar el frame de la cámara")
        break

    # Mostrar la imagen de la cámara en vivo
    cv2.imshow('Cámara en Vivo', frame)

    # Esperar por una tecla durante 1 ms
    key = cv2.waitKey(1) & 0xFF

    # Si se presiona la tecla 'p' (puedes cambiarla por otra)
    if key == ord('p'):
        # Procesar la imagen actual
        bright_image, blurred, edges, contours_image, contours = process_image(frame)

        # Detectar frutas o evaluar si hay algo relevante
        if contours:
            # Calcular rugosidad
            rugosity = calculate_rugosity(bright_image)

            # Calcular características de forma
            shape_features = calculate_shape_features(contours)

            # Calcular características GLCM
            contrast, correlation, energy, homogeneity = calculate_glcm_features(bright_image)

            # Obtener rangos de color
            color_ranges = get_color_ranges(bright_image, contours)

            # Verificar que se hayan obtenido los rangos de color
            if shape_features and color_ranges:
                # Crear un diccionario con todas las características
                features = {
                    'rugosity': rugosity,
                    'number_of_contours': shape_features['number_of_contours'],
                    'area': shape_features['area'],
                    'perimeter': shape_features['perimeter'],
                    'aspect_ratio': shape_features['aspect_ratio'],
                    'height': shape_features['height'],
                    'width': shape_features['width'],
                    'circularity': shape_features['circularity'],
                    'mean_normalized_contour_x': shape_features['mean_normalized_contour_x'],
                    'mean_normalized_contour_y': shape_features['mean_normalized_contour_y'],
                    'std_normalized_contour_x': shape_features['std_normalized_contour_x'],
                    'std_normalized_contour_y': shape_features['std_normalized_contour_y'],
                    'contrast': contrast,
                    'correlation': correlation,
                    'energy': energy,
                    'homogeneity': homogeneity
                }
                # Añadir Momentos Hu
                for i, hu_moment in enumerate(shape_features['hu_moments']):
                    features[f'hu_moment_{i+1}'] = hu_moment

                # Añadir los rangos de color
                for i in range(1, 6):
                    if i <= len(color_ranges):
                        h_range = color_ranges[i - 1]
                        features[f'hue_range_{i}_start'] = h_range[0]
                        features[f'hue_range_{i}_end'] = h_range[1]
                    else:
                        # Si hay menos de 5 rangos, completar con ceros
                        features[f'hue_range_{i}_start'] = 0
                        features[f'hue_range_{i}_end'] = 0

                # Convertir las características en un DataFrame
                X_new = pd.DataFrame([features])

                # Asegurarse de que las columnas estén en el orden correcto y que todas las columnas necesarias estén presentes
                expected_features = clf_loaded.feature_names_in_
                missing_features = set(expected_features) - set(X_new.columns)
                for feature in missing_features:
                    X_new[feature] = 0  # O algún valor por defecto apropiado

                # Reordenar las columnas
                X_new = X_new[expected_features]

                # Realizar la predicción
                prediction_encoded = clf_loaded.predict(X_new)
                prediction = le.inverse_transform(prediction_encoded)

                # Mostrar resultados
                print(f"\nPredicción: {prediction[0]}")
                print(f"Rugosidad: {rugosity:.2f}")
                print(f"Área: {shape_features['area']}, Perímetro: {shape_features['perimeter']}, "
                      f"Aspect Ratio: {shape_features['aspect_ratio']}, Circularidad: {shape_features['circularity']}")
                for i, hu_moment in enumerate(shape_features['hu_moments']):
                    print(f"Momento Hu {i + 1}: {hu_moment}")
                print(f"Contrast: {contrast}, Correlation: {correlation}, Energy: {energy}, Homogeneity: {homogeneity}")
                print("Rangos de color (Hue):")
                for i in range(1, 6):
                    print(f"Hue Range {i}: {features[f'hue_range_{i}_start']} - {features[f'hue_range_{i}_end']}")

                # Mostrar imagen con contornos
                cv2.imshow('Procesado', contours_image)
            else:
                print("No se pudieron obtener todas las características necesarias.")
        else:
            print("No se detectaron contornos en la imagen.")

    # Presionar 'q' para salir del bucle
    if key == ord('q'):
        break

# Liberar la cámara y cerrar ventanas
cap.release()
cv2.destroyAllWindows()
