#!/bin/bash

WEBOTS_PATH="/usr/local/bin/webots"

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=600
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=800
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=1000
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=1200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=1400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1600 --test_model2=1600
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1400 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1400 --test_model2=200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1400 --test_model2=400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1400 --test_model2=600
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1400 --test_model2=800
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1400 --test_model2=1000
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1400 --test_model2=1200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1400 --test_model2=1400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1200 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1200 --test_model2=200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1200 --test_model2=400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1200 --test_model2=600
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1200 --test_model2=800
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1200 --test_model2=1000
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1200 --test_model2=1200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1000 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1000 --test_model2=200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1000 --test_model2=400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1000 --test_model2=600
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1000 --test_model2=800
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=1000 --test_model2=1000
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=800 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=800 --test_model2=200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=800 --test_model2=400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=800 --test_model2=600
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=800 --test_model2=800
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=600 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=600 --test_model2=200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=600 --test_model2=400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=600 --test_model2=600
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=400 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=400 --test_model2=200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=400 --test_model2=400
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=200 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=200 --test_model2=200
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=0 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5
