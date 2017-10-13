MODEL_URL='http://storage.googleapis.com/download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz'
MODEL_FILE_FOLDER='ros/src/tl_detector/light_classification/'
MODEL_FILE='ros/src/tl_detector/light_classification/faster_rcnn_resnet101_coco_11_06_2017.tar.gz'
wget -P $MODEL_FILE_FOLDER $MODEL_URL
tar -xvzf $MODEL_FILE -C $MODEL_FILE_FOLDER