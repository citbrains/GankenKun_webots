#!/bin/bash

WEBOTS_PATH="/usr/local/bin/webots"

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=150 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=150 --test_model2=30
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=150 --test_model2=60
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=150 --test_model2=90
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=150 --test_model2=120
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=150 --test_model2=150
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=120 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=120 --test_model2=30
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=120 --test_model2=60
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=120 --test_model2=90
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=120 --test_model2=120
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=90 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=90 --test_model2=30
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=90 --test_model2=60
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=90 --test_model2=90
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=60 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=60 --test_model2=30
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=60 --test_model2=60
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=30 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=30 --test_model2=30
pkill -f $WEBOTS_PATH
sleep 5

$WEBOTS_PATH --mode=fast --no-rendering --batch worlds/mat_train.wbt &
python controllers/mat_train/mat_train.py --test_model1=0 --test_model2=0
pkill -f $WEBOTS_PATH
sleep 5
