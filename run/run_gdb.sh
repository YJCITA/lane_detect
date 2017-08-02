cd /home/yj/src/CV/Lane/LaneDetection/build/log
./clean.sh # clean log file
cd -
cd ../

gdb /home/yj/src/CV/Lane/LaneDetection/build/bin/lane_detect    #2>./build/error.txt

# 下面这个的手动额外输入
set args --flagfile "/home/yj/bak/data/lane/data_develop/2/detect_lane.flag"
