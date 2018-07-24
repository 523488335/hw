import requests
import json
from exception.HwExpection import *
import serial


class SoftUpdate(object):
    # 服务器ip和端口配置
    # ip = '39.108.174.160'
    ip = 'localhost'
    port = '8080'

    def __init__(self, project, version):
        super().__init__()
        self.version = version
        self.projectName = project
        # 获取更新信息并存储在updateMap属性中
        resp = requests.get("http://" + SoftUpdate.ip + ":" + SoftUpdate.port + "/file/update",
                            params={'projectName': self.projectName})
        # 响应状态为200则正常，反之异常
        if resp.status_code != 200:
            raise HwException('服务器异常code:' + str(resp.status_code))
        self.updateMap = json.loads(resp.text)
        print(self.updateMap)

    # 根据版本号判断是否更新
    def is_update(self):
        curr_arr = self.version.split('.')
        new_arr = self.updateMap['version'].split('.')
        for c, n in zip(curr_arr, new_arr):
            if c > n:
                return False
            elif c < n:
                return True
        return False

    def update(self):
        if self.updateMap:
            resp = requests.get("http://" + SoftUpdate.ip + ":" + SoftUpdate.port + "/file/download",
                                params={'filename': self.updateMap['path']}, stream=True)
            # 响应状态为200则正常，反之异常
            if resp.status_code != 200:
                raise HwException('服务器异常code:' + str(resp.status_code))
            # 保存为文件
            if 0:
                # 用文件路径截取文件后缀
                suffix = self.updateMap['path'].split('.')
                if len(suffix) > 1:
                    suffix = suffix[len(suffix) - 1]
                else:
                    suffix = ''
                # 用文件项目名版本号和后缀拼接文件名
                filename = self.projectName + self.updateMap['version'] + suffix
                # 下载文件到本地
                file = open('F:/' + filename, 'wb')
                for chunk in resp.iter_content(chunk_size=512):
                    if chunk:
                        file.write(chunk)
            SoftUpdate.send_scm(resp.content)
            return True
        return False

    # 更新文件发送给下位机
    @staticmethod
    def send_scm(file):
        print("=====" + str(file))
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        ser.write(file)
        ser.close()


if __name__ == '__main__':
    print(SoftUpdate('OLED', '0.0.0').update())





