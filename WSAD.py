
import time
import keyboard
from pynput import keyboard as kb
from threading import Lock
import xsensdot_pc_sdk
from user_settings import *
from collections import defaultdict

waitForConnections = True
exit_flag=False

class CallbackHandler(xsensdot_pc_sdk.XsDotCallback):
    def __init__(self, max_buffer_size=5):
        xsensdot_pc_sdk.XsDotCallback.__init__(self)
        self.m_detectedDots = list()
        self.m_errorReceived = False
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = defaultdict(list)
        self.m_lock = Lock()

    def getDetectedDots(self):
        return self.m_detectedDots

    def errorReceived(self):
        return self.m_errorReceived

    def packetsAvailable(self):
        for dev in self.m_detectedDots:
            if self.packetAvailable(dev.bluetoothAddress()) == 0:
                return False
        return True

    def packetAvailable(self, bluetoothAddress):
        self.m_lock.acquire()
        res = len(self.m_packetBuffer[bluetoothAddress]) > 0
        self.m_lock.release()
        return res

    def getNextPacket(self, bluetoothAddress):
        if len(self.m_packetBuffer[bluetoothAddress]) == 0:
            return None
        self.m_lock.acquire()
        oldest_packet = xsensdot_pc_sdk.XsDataPacket(self.m_packetBuffer[bluetoothAddress].pop(0))
        self.m_lock.release()
        return oldest_packet

    def onAdvertisementFound(self, port_info):
        if not whitelist or port_info.bluetoothAddress() in whitelist:
            self.m_detectedDots.append(port_info)
        else:
            print(f"Ignoring {port_info.bluetoothAddress()}")

    def onBatteryUpdated(self, dev, batteryLevel, chargingStatus):
        print(dev.deviceTagName() + f" BatteryLevel: {batteryLevel} Charging status: {chargingStatus}")

    def onError(self, errorString):
        print(f"Error received: {errorString}")
        self.m_errorReceived = True

    def onLiveDataAvailable(self, dev, pack):
        self.m_lock.acquire()
        while len(self.m_packetBuffer[dev.portInfo().bluetoothAddress()]) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer[dev.portInfo().bluetoothAddress()].pop()
        self.m_packetBuffer[dev.portInfo().bluetoothAddress()].append(xsensdot_pc_sdk.XsDataPacket(pack))
        self.m_lock.release()
def on_press(key):
    global exit_flag
    if key == kb.Key.esc:
        exit_flag = True

if __name__ == "__main__":
    # Print SDK version
    version = xsensdot_pc_sdk.XsVersion()
    xsensdot_pc_sdk.xsdotsdkDllVersion(version)
    print(f"Using Xsens DOT SDK version: {version.toXsString()}")

    # Create connection manager
    manager = xsensdot_pc_sdk.XsDotConnectionManager()
    if manager is None:
        print("Manager could not be constructed, exiting.")
        exit(-1)

    # Create and attach callback handler to connection manager
    callback = CallbackHandler()
    manager.addXsDotCallbackHandler(callback)

    # Start a scan and wait until we have found one or more DOT Devices
    print("Scanning for devices...")
    manager.enableDeviceDetection()

    # Setup the keyboard input listener
    listener = kb.Listener(on_press=on_press)
    listener.start()

    print("Press any key or wait 20 seconds to stop scanning...")
    connectedDOTCount = 0
    startTime = xsensdot_pc_sdk.XsTimeStamp_nowMs()
    while waitForConnections and not callback.errorReceived() and xsensdot_pc_sdk.XsTimeStamp_nowMs() - startTime <= 15000 and not exit_flag:
        time.sleep(0.1)

        nextCount = len(callback.getDetectedDots())
        if nextCount != connectedDOTCount:
            print(f"Number of connected DOTs: {nextCount}. Press any key to start.")
            connectedDOTCount = nextCount

    manager.disableDeviceDetection()
    print("Stopped scanning for devices.")

    if len(callback.getDetectedDots()) == 0:
        print("No Xsens DOT device(s) found. Aborting.")
        exit(-1)

    deviceList = list()
    sequence_number = 0
    for portInfo in callback.getDetectedDots():
        address = portInfo.bluetoothAddress()

        print(f"Opening DOT with address: @ {address}")
        if not manager.openPort(portInfo):
            print(f"Connection to Device {address} failed, retrying...")
            print(f"Device {address} retry connected:")
            if not manager.openPort(portInfo):
                print(f"Could not open DOT. Reason: {manager.lastResultText()}")
                continue

        device = manager.device(portInfo.deviceId())
        if device is None:
            continue

        deviceList.append(device)
        print(f"Found a device with Tag: {device.deviceTagName()} @ address: {address}")

        filterProfiles = device.getAvailableFilterProfiles()
        print("Available filter profiles:")
        for f in filterProfiles:
            print(f.label())

        print(f"Current profile: {device.onboardFilterProfile().label()}");
        if device.setOnboardFilterProfile("General"):
            print("Successfully set profile to General");
        else:
            print("Setting filter profile failed!");

        print("Setting quaternion CSV output")
        device.setLogOptions(xsensdot_pc_sdk.XsLogOptions_Quaternion)

        print("Putting device into measurement mode.")
        if not device.startMeasurement(xsensdot_pc_sdk.XsPayloadMode_ExtendedEuler):
            print(f"Could not put device into measurement mode. Reason: {manager.lastResultText()}")
            continue
    print("\nMain loop. Recording data for 10 seconds.")
    print("-----------------------------------------")

    s = ""
    for device in deviceList:
        s += f"{device.portInfo().bluetoothAddress():42}"
        
    print("%s" % s, flush=True)

    orientationResetDone = False
    startTime = xsensdot_pc_sdk.XsTimeStamp_nowMs()
    while xsensdot_pc_sdk.XsTimeStamp_nowMs() - startTime <= 120000 and not exit_flag:
        if exit_flag:
            break
        if callback.packetsAvailable():
            s = ""
            for device in deviceList:
                # Retrieve a packet
                packet = callback.getNextPacket(device.portInfo().bluetoothAddress())
                # print(packet)
                if packet.containsOrientation():
                    euler = packet.orientationEuler()
                    Roll=f"{euler.x():7.2f}"
                    pitch=f"{euler.y():7.2f}"

                    s += f"Roll:{euler.x():7.2f}, Pitch:{euler.y():7.2f}, Yaw:{euler.z():7.2f}| "

                    if(float(pitch) >= 20 and float(pitch)<= 180):
                        keyboard.press_and_release('Up')
                        if(float(Roll)>=20 and float(Roll)<=180):
                            keyboard.press_and_release('Right')
                        elif(float(Roll)<=-20 and float(Roll)>=-180):
                            keyboard.press_and_release('Left')
                        else:
                            pass
                    
                    elif(float(pitch)<=-20 and float(pitch)>=-180):
                        keyboard.press_and_release('Down')
                        if(float(Roll)>=20 and float(Roll)<=180):
                            keyboard.press_and_release('Right')
                        elif(float(Roll)<=-20 and float(Roll)>=-180):
                            keyboard.press_and_release('Left')
                        else:
                            pass

                    else:
                        if(float(Roll)>=20 and float(Roll)<=180):
                            keyboard.press_and_release('Right')
                        elif(float(Roll)<=-20 and float(Roll)>=-180):
                            keyboard.press_and_release('Left')
                        else:
                            pass

            print("%s\r" % s, end="", flush=True)

            if not orientationResetDone and xsensdot_pc_sdk.XsTimeStamp_nowMs() - startTime > 60000:
                for device in deviceList:
                    print(f"\nResetting heading for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
                    if device.resetOrientation(xsensdot_pc_sdk.XRM_Heading):
                        print("OK", end="", flush=True)
                    else:
                        print(f"NOK: {device.lastResultText()}", end="", flush=True)
                print("\n", end="", flush=True)
                orientationResetDone = True
    print("\n-----------------------------------------", end="", flush=True)

    for device in deviceList:
        print(f"\nResetting heading to default for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
        if device.resetOrientation(xsensdot_pc_sdk.XRM_DefaultAlignment):
            print("OK", end="", flush=True)
        else:
            print(f"NOK: {device.lastResultText()}", end="", flush=True)
    print("\n", end="", flush=True)

    print("\nStopping measurement...")
    for device in deviceList:
        if not device.stopMeasurement():
            print("Failed to stop measurement.")
    print("Closing ports...")
    manager.close()

    print("Successful exit.")