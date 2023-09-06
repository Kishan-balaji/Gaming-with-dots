from xdpchandler import *
exit_flag=False

class movelladot():
    def initiate(self):
        self.xdpcHandler = XdpcHandler()

        if not self.xdpcHandler.initialize():
            self.xdpcHandler.cleanup()
            exit(-1)

        self.xdpcHandler.scanForDots()
        if len(self.xdpcHandler.detectedDots()) == 0:
            print("No Movella DOT device(s) found. Aborting.")
            self.xdpcHandler.cleanup()
            exit(-1)

        self.xdpcHandler.connectDots()

        if len(self.xdpcHandler.connectedDots()) == 0:
            print("Could not connect to any Movella DOT device(s). Aborting.")
            self.xdpcHandler.cleanup()
            exit(-1)

        for device in self.xdpcHandler.connectedDots():
            filterProfiles = device.getAvailableFilterProfiles()
            print("Available filter profiles:")
            for f in filterProfiles:
                print(f.label())

            print(f"Current profile: {device.onboardFilterProfile().label()}")
            if device.setOnboardFilterProfile("General"):
                print("Successfully set profile to General")
            else:
                print("Setting filter profile failed!")


            print("Putting device into measurement mode.")
            if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_ExtendedEuler):
                print(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
                continue
    def start(self):
        print("\nMain loop. Recording data for 10 seconds.")
        print("-----------------------------------------")
        orientationResetDone = False
        startTime = movelladot_pc_sdk.XsTimeStamp_nowMs()
        # while movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime <= 10000:
        while not exit_flag:
            if self.xdpcHandler.packetsAvailable():
                s = ""
                devicelist=self.xdpcHandler.connectedDots()
                for device in devicelist:
                    packet = self.xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())
                    if packet.containsOrientation():
                        euler = packet.orientationEuler()
                        Roll=f"{euler.x():7.2f}"
                        s += f"Roll:{Roll}"   
                        if (float(Roll) <= 20 and float(Roll) >= -20):
                            pass
                        elif(float(Roll) >= -180 and float(Roll)<= -20):
                            keyboard.press_and_release('Left')
                        elif(float(Roll) >=20 and float(Roll) <= 180):
                            keyboard.press_and_release('Right')
                print("%s\r" % s, end="", flush=True)

                # if not orientationResetDone and movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime > 6000:
                #     for device in self.xdpcHandler.connectedDots():
                #         print(f"\nResetting heading for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
                #         if device.resetOrientation(movelladot_pc_sdk.XRM_Heading):
                #             print("OK", end="", flush=True)
                #         else:
                #             print(f"NOK: {device.lastResultText()}", end="", flush=True)
                #     print("\n", end="", flush=True)
                #     orientationResetDone = True
        print("\n-----------------------------------------", end="", flush=True)
    def stopmeaurement(self):
        for device in self.xdpcHandler.connectedDots():
            print(f"\nResetting heading to default for device {device.portInfo().bluetoothAddress()}: ", end="", flush=True)
            if device.resetOrientation(movelladot_pc_sdk.XRM_DefaultAlignment):
                print("OK", end="", flush=True)
            else:
                print(f"NOK: {device.lastResultText()}", end="", flush=True)
        print("\n", end="", flush=True)

        print("\nStopping measurement...")
        for device in self.xdpcHandler.connectedDots():
            if not device.stopMeasurement():
                print("Failed to stop measurement.")

        self.xdpcHandler.cleanup()




if __name__ == "__main__":
    try:
        from pynput import keyboard as kb
        exit_flag=False
        def on_press(key):
            global exit_flag
            if key == kb.Key.esc:
                exit_flag = True
            
        listener = kb.Listener(on_press=on_press)
        listener.start()
        d=movelladot()
        d.initiate()
        d.start()
    finally:
        d.stopmeaurement()