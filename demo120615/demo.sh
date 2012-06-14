hrpsys-simulator SampleSV_RangeSensor.xml -endless &
CameraImageViewerComp -f rtc.conf2 &

sleep 1

python connect.py

echo "To finish demo, hit return key"
read ans

killall CameraImageViewerComp hrpsys-simulator
