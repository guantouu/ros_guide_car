from pypozyx import (PozyxConstants, get_first_pozyx_serial_port, PozyxSerial,
                     DeviceCoordinates, Coordinates, POZYX_POS_ALG_UWB_ONLY, POZYX_2D,
                     POZYX_SUCCESS, version,  get_first_pozyx_serial_port, SingleRegister,
                     DeviceList, PozyxRegisters, EulerAngles, Acceleration ,POZYX_POS_ALG_TRACKING)  
import rospy
from time import sleep
class uwbProcessing():
    def __init__(self, pozyx, anchor, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_2D, 
                 height=50, remote_id=None):
        self.pozyx = pozyx
        self.anchors = anchor
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id
        self.sensitive = 400

    def setup(self):
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")
        print("- System will manually configure tag")
        print("")
        print("- System will auto start positioning")
        print("")
        if self.remote_id is None:
            self.pozyx.printDeviceInfo(self.remote_id)
        else:
            for device_id in [None, self.remote_id]:
                 self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")

#       self.pozyx.setPositionFilter(PozyxConstants.FILTER_TYPE_MOVING_AVERAGE , 10 ,remote_id = None)
        self.setAnchorsManual(save_to_flash=False)
        self.printPublishConfigurationResult()

    def printPublishConfigurationResult(self):
        list_size = SingleRegister()
        
        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            pass
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            
    def setAnchorsManual(self, save_to_flash=False): #Anchor
        status = self.pozyx.clearDevices(remote_id=self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
                                                       remote_id=self.remote_id)
        if save_to_flash:
            self.pozyx.saveAnchorIds(remote_id=self.remote_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
        return status

if __name__ == "__main__":
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()
    remote_id = 0x6740
    remote = False
    if not remote:
        remote_id = None

    use_processing = False  # enable to send position data through OSC
    osc_udp_client = None
    ip = "127.0.0.1" # configure if you want to route OSC to outside your localhost. Networking knowledge is required.
    network_port = 22

    pozyx = PozyxSerial(serial_port)
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
    dimension = PozyxConstants.DIMENSION_2D
    hight = 50
    #necessary data for calibration, change the IDs and coordinates yourself according to your measurement
    anchors = [DeviceCoordinates(26426, 1, Coordinates(-(-5177 - 1073), 4781 + 3543 , 0)),
            DeviceCoordinates(26381, 1, Coordinates(-(-5750 - 1073), -3316 + 3543,  0)),
            DeviceCoordinates(26386, 1, Coordinates(1123 - 1073, 5479 + 3543, 0)),
            DeviceCoordinates(26429, 1, Coordinates(1073 - 1073, -3543 + 3543 ,  0))]

    r = uwbProcessing(pozyx, anchors, algorithm, dimension, hight, remote_id=None)
    r.setup()