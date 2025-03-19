import os
import time
import logging
from motor_cmd import FormSendData, ParseRecvData
import can

# 配置日志记录
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class Can:
    def __init__(self, interface_name="can0", bitrate=1000000):
        self.interface_name = interface_name
        self.bitrate = bitrate
        self.id1 = 1 # 默认关节的id为1，若与实际的关节id不符，请将self.id1修改为实际的关节id
        self.can0 = None
        self.setup_can_interface()

    def setup_can_interface(self):
        try:
            # 关闭已存在的 CAN 接口
            os.system(f'sudo ifconfig {self.interface_name} down')
            # 配置 CAN 接口
            os.system(f'sudo ip link set {self.interface_name} up type can bitrate {self.bitrate}')
            os.system(f'sudo ifconfig {self.interface_name} up')
            # 创建 CAN 总线
            self.can0 = can.interface.Bus(channel=self.interface_name, interface='socketcan')
            logging.info(f"CAN interface '{self.interface_name}' initialized successfully.")
        except Exception as e:
            logging.error(f"Failed to initialize CAN interface: {e}")
            raise

    def can_func(self):
        i = 0
        while True:
            print(i)
            i += 1
            #time.sleep(0.2)  # 增加发送间隔
            if i == 1:
                self.can_send(cmd=1)
            elif i == 2:
                self.can_send(cmd=2)
            else:
                self.can_send(cmd=4)
            self.can_recv(cmd=4)

    def can_send(self, cmd):
        try:
            data_send_can = FormSendData(cmd=cmd, id=self.id1, data_send_orgin=[5, 0, 0, 5.0, 0])
            #logging.info(f"Sending CAN message: {data_send_can}")
            if cmd in [1, 2]:  # 控制命令
                msg = can.Message(arbitration_id=data_send_can[0], is_extended_id=False)
            else:  # 运动命令
                msg = can.Message(arbitration_id=data_send_can[0], data=data_send_can[2:], is_extended_id=False)

            # 检查缓冲区是否已满
            while True:
                try:
                    self.can0.send(msg)
                    break
                except can.CanOperationError as e:
                    if "No buffer space available" in str(e):
                        logging.warning("CAN buffer is full. Waiting...")
                        time.sleep(0.1)  # 等待缓冲区可用
                        break
        except Exception as e:
            logging.error(f"Failed to send CAN message: {e}")

    def can_recv(self, cmd):
        try:
            frame = self.can0.recv(0.003)  # 超时设置为 3ms
            if frame is None:
                logging.warning("No CAN message received within timeout.")
                return
            data = list(frame.data)
            can_id = (cmd << 5) | (self.id1 + 0x10)
            data_recv_can = ParseRecvData(id=can_id, data_recv_orgin=data)
            logging.info(f"Received CAN message: {data_recv_can}")
        except Exception as e:
            logging.error(f"Failed to receive CAN message: {e}")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            if self.can0:
                self.can0.shutdown()
            logging.info(f"CAN interface '{self.interface_name}' shut down successfully.")
        except Exception as e:
            logging.error(f"Failed to shut down CAN interface: {e}")

if __name__ == "__main__":
    with Can(interface_name="can0", bitrate=1000000) as a:
        a.can_func()
