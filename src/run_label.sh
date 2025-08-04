#ng
python3 label_gst.py \
    -f 2 \
    -ml yolov5 \
    -m /opt/yolov5x-int8.tflite \
    -l /opt/yolov8.labels \
    -k "YOLOv5,q-offsets=<1.0>,q-scales=<0.0044934190809726715>" \
    --use_dsp

