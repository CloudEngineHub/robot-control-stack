from simpub.xr_device.meta_quest3 import MetaQuest3
from quest_iris_dual_arm import MySimPublisher, MySimScene, MQ3_ADDR


MySimPublisher(MySimScene(), MQ3_ADDR)
reader = MetaQuest3("RCSNode")
while True:
    data = reader.get_controller_data()
    print(data)