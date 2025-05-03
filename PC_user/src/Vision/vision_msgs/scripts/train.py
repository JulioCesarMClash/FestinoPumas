from ultralytics import YOLO

# Load a model
#model = YOLO("yolo8n.yaml")  # build a new model from YAML
model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)
#model = YOLO("yolo8n.yaml").load("yolo8n.pt")  # build from YAML and transfer weights

# Train the model
results = model.train(
    data="yolo_json/YOLODataset/dataset.yaml",
    epochs=100,          # Número de épocas
    batch=16,            # Tamaño del batch (depende de tu GPU)
    imgsz=640,           # Tamaño de imagen
    device=0,            # 0 para GPU, "cpu" para CPU
    workers=8,           # Núcleos para carga de datos
    optimizer="auto",    # Optimizador (adam, sgd, etc.)
    lr0=0.01,            # Tasa de aprendizaje inicial
    name="yolov8_custom" # Nombre del experimento
)