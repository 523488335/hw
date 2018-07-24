class HwException(Exception):
    err_un_know = 0x0000
    err_ill_args = 0x0001
    err_process = 0x0002

    err_hardware = 0x0010

    def __init__(self, msg, err_code=err_un_know):
        super().__init__()
        self.err_code = err_code
        self.msg = msg

    def __str__(self):
        return 'errCode:' + str(self.err_code) + ',msg:' + self.msg
