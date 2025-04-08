from ultralytics import YOLO

model = YOLO('/home/yangjeongwon/Downloads/batch_16_epoch_20_1000.pt')

trt_model = model.export (format="engine")
