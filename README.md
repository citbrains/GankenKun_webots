# GankenKun_webots

RoboCup2021関連の情報
https://github.com/citbrains/GankenKun_webots/wiki/RoboCup2021%E9%96%A2%E9%80%A3%E6%83%85%E5%A0%B1  

protocol buffersのcppを入れないとエラーになります。あとVrepの環境構築scriptの内容も実行しておくべきです。

protocol buffersのインストール方法  
wget https://github.com/protocolbuffers/protobuf/releases/download/v3.15.8/protobuf-cpp-3.15.8.tar.gz  
tar -xvzf protobuf-cpp-3.15.8.tar.gz  
解答した所に移動  
./configure  
make -j8  
make check  
sudo make install  
sudo ldconfig  
