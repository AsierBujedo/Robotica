import cv2
import numpy as np
import mahotas as mh
import math



#Ajustar el brillo y el contraste
def adjust_brightness_contrast(image, alpha=1, beta=70):
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

#Uso de filtros gaussiano y canny
def process_image(image):
    bright_image = adjust_brightness_contrast(image)
    
    #Preprocesamiento
    blurred = cv2.GaussianBlur(bright_image, (5, 5), 0)
    edges = cv2.Canny(blurred, 100, 200)

    
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Crear una copia para dibujar contornos
    output = image.copy()
    cv2.drawContours(output, contours, -1, (0, 255, 0), 2)  # Contornos en verde

    return bright_image, blurred, edges, output, contours


# Calcular rugosidad 
def calculate_rugosity(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    rugosity = np.var(gray)  # Varianza de la imagen
    return rugosity


#Calcular caracteristicas de forma
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
    normalized_contour = normalized_contour / [w, h]  

    # Calcular estadísticas de las coordenadas normalizadas
    mean_coord = np.mean(normalized_contour, axis=0).flatten()
    std_coord = np.std(normalized_contour, axis=0).flatten()

    # Número de contornos
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


#Calcular características GLCM
def calculate_glcm_features(image):
    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    #Calcular las características GLCM usando mahotas
    haralick_features = mh.features.texture.haralick(gray, distance=1).mean(axis=0)
    
    # Extraer las características necesarias
    contrast = haralick_features[1]       # contrast
    correlation = haralick_features[2]    # correlation
    energy = haralick_features[8]         # energy
    homogeneity = haralick_features[4]    # homogeneity
    
    return contrast, correlation, energy, homogeneity


# Obtener los rangos de los colores
def get_color_ranges(image, contours):
    #comporbar si hay contornos
    if len(contours) == 0:
        return None  

    # Crear una máscara vacía
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    
    # Procesar cada contorno
    for cnt in contours:
        hull = cv2.convexHull(cnt)
        cv2.drawContours(mask, [hull], -1, (255), thickness=cv2.FILLED)  
    
    # Aplicar máscara y convertir a HSV
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    hsv_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
    
    # Filtrar el color blanco y tonos claros
    lower_bound = np.array([0, 50, 0])   
    upper_bound = np.array([180, 255, 230])  # Excluir blanco
    filtered_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    
    # Excluir el azul
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