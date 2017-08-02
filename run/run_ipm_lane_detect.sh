cd /home/yj/src/CV/Lane/LaneDetection/build/log
./clean.sh # clean log file
cd -
cd ../

/home/yj/src/CV/Lane/LaneDetection/build/bin/lane_detect  --flagfile "/home/yj/bak/data/lane/data_develop/2/detect_lane.flag"  #2>./build/error.txt


