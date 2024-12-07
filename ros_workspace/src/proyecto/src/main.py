import cv2
from vision import *
from model_predictor import *
from controller import *

# Cargar modelos
clf, le = load_models('random_forest_model.pkl', 'label_encoder.pkl')

# Abrir cámara
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    exit()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo capturar el frame.")
            break

        cv2.imshow('Camara en vivo', frame)
        key = cv2.waitKey(1) & 0xFF
        
        # Procesar imagen al presionar la tecla 'c'
        if key == ord('c'):  
            bright_image, blurred, edges, contours_image, contours = process_image(frame)

            if contours:
                # Extraer características
                rugosity = calculate_rugosity(bright_image)
                shape_features = calculate_shape_features(contours)
                contrast, correlation, energy, homogeneity = calculate_glcm_features(bright_image)
                color_ranges = get_color_ranges(bright_image, contours)

                if shape_features and color_ranges:
                    # Crear diccionario de características
                    features = {**shape_features, 'rugosity': rugosity, 'contrast': contrast, 'correlation': correlation,
                                'energy': energy, 'homogeneity': homogeneity}
                    
                    for i, h_range in enumerate(color_ranges[:5]):
                        features[f'hue_range_{i+1}_start'] = h_range[0]
                        features[f'hue_range_{i+1}_end'] = h_range[1]

                    #Predecir clase
                    prediction = predict_class(clf, le, features)
                    print(f"Predicción: {prediction}")

                    # Mover fruta
                    if "defecto" in prediction:
                        controller.poner_caja_mala()
                    else:
                        controller.poner_caja_buena()

            else:
                print("No se detectaron contornos.")
        
        # Salir al presionar 'q'
        elif key == ord('q'):  
            break

except Exception as e:
    print(f"Se ha produccido un error durante la ejecución: {e}")

finally:
    #liberar el recurso de la cámara y cerra ventanas
    cap.release()
    cv2.destroyAllWindows()
    print("Cámara liberada y ventanas cerradas.")
