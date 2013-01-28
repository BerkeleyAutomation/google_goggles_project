#!/bin/bash

IMAGE_DIR=$(rospack find google_goggles)/data/images_dense
SAVE_DIR=$(rospack find google_goggles)/data/tests/dense/random_100-to-7000_$(date +%Y-%m-%d)

mkdir -p $SAVE_DIR

CLEAR_FIRST=--clear-first

cmd() {
rosrun google_goggles test_images_new.py $CLEAR_FIRST --name random$1 --save-dir $SAVE_DIR -t $IMAGE_DIR/training/all -v $IMAGE_DIR/unorganized --num-samples $1 --sample-all --num-rounds 1 --dont-validate-training-images
}

cmd 100 && cmd 200 && cmd 300 && cmd 400 && cmd 500 && cmd 750 && cmd 1000 && cmd 1500 && cmd 2000 && cmd 2500 && cmd 3000 && cmd 4000 && cmd 5000 && cmd 6000 && cmd 7000
