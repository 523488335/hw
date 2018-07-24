import time
import subprocess

start = time.time() * 1000
file = open('/home/shang/Downloads/screen2.png', 'wb')
# p0 = subprocess.Popen('adb shell screenrecord --bit-rate 2000000  --time-limit 10 /sdcard/test.mp4',
#                       shell=True, stdout=subprocess.PIPE)
p1 = subprocess.Popen('adb shell screencap -p', shell=True, stdout=subprocess.PIPE)
p2 = subprocess.Popen("sed 's/\r$//'", shell=True, stdin=p1.stdout, stdout=subprocess.PIPE)
file.write(p2.stdout.read())
end = time.time() * 1000
print(str(end - start) + "ms")

