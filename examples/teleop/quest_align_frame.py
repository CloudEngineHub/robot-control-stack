from quest_iris_dual_arm import MQ3_ADDR, MySimPublisher, MySimScene
from simpub.xr_device.meta_quest3 import MetaQuest3

MySimPublisher(MySimScene(), MQ3_ADDR)
reader = MetaQuest3("RCSNode")
while True:
    data = reader.get_controller_data()
    # print(data)
