#!/bin/bash

IMAGE_DIR=$(rospack find google_goggles)/data/images
SAVE_DIR=$(rospack find google_goggles)/data/tests

mkdir -p $SAVE_DIR

rosrun google_goggles test_images.py --clear-first --name random228_timed --save-dir $SAVE_DIR -t $IMAGE_DIR/training/all_no_ketchup -v $IMAGE_DIR/validation_no_ketchup --num-samples 228 --sample-all --num-rounds 1 --dont-validate-training-images && \
\
rosrun google_goggles test_images.py --clear-first --name test2_timed --save-dir $SAVE_DIR -t $IMAGE_DIR/training/test2_no_ketchup -v $IMAGE_DIR/validation_no_ketchup --train-all --round-dirs --dont-validate-training-images && \
\
rosrun google_goggles test_images.py --clear-first --name test2_2_timed --save-dir $SAVE_DIR -t $IMAGE_DIR/training/test2_no_ketchup/round2 -v $IMAGE_DIR/validation_no_ketchup --train-all --dont-validate-training-images --remove-from-validation-dir $IMAGE_DIR/training/test2_no_ketchup && \
\
rosrun google_goggles test_images.py --clear-first --name test2_3_timed --save-dir $SAVE_DIR -t $IMAGE_DIR/training/test2_no_ketchup/round3 -v $IMAGE_DIR/validation_no_ketchup --train-all --dont-validate-training-images --remove-from-validation-dir $IMAGE_DIR/training/test2_no_ketchup

