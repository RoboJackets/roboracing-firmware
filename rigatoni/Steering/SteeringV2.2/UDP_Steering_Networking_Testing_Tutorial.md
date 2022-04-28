Tutorial on how to test sending steering data over UDP from your laptop to the Steering Board:
by Oankar Patil

Assuming windows 10 laptop:


1. Download and Install "Packet Sender" from the internet. Close it if it launches after installing. Disconnect from the internet. Ensure there is power to the Steering Board. Plug your ethernet cable into your computer (use a USB to Ethernet adaptor if needed) and connect it to the steering board.

2. Go to Control Panel. Click "Network and Internet". Click "Network and Sharing Center". Click "CHange Adaptor Settings".

3. Right click on the new Ethernet option (usually called Ethernet2 and should say something about initializing). Click Properties.

4. A pop up box should show up. Click on "Internet Protocol Version 4 (TCP/IPv4).

5. Click Properties on the bottom right. Another pop up box should show up.

6. In General, change the selection from "Obtain an IP Address Automatically" to "Use the following IP Address".

7. In the IP Address block, write: 192.168.20.12
    * the last two (20 and 12) are up to the user, but they should each be above 10 and less than 250
    * You can click on each block subsection to write the number (ex: click on the last block and write 12).

8. Click on the Subnet Mask box and it should auto-fill.

9. For Preferred DNS Server, write: 1.1.1.1 (This doesn't matter, you just need some DNS server and this is easy to remember)

10. Click OK and close all the pop up boxes.

11. Open Packet Sender. The address should be Steering's IP address (192.168.20.5) (You can find other board IP addresses in RJ Net Mbed UDP.h). The Port should be 8888. The second block next to "Resend Delay" should say UDP. If it says TCP or anything else, click on it and change it to UDP. At the very bottom of packet sender, there should be a leftmost yellow symbol block that says UDP and has a string of numbers next to it.
Click on it and change the numebr to 8888. Packet Sender has now been setup.

12. Head into main.cpp in SteeringV2.2. Ensure that the IP address for laptopIP matches the one you wrote in step 7.

13. In all of the firmware, any references to nucIP (use ctrl-f to find all references) should be changed to refer to laptopIP.

14. Save and flash your firmware. Let it do any calibration and homing setup.

15. Once it has finished homing and is in center, go back to Packet Sender. In the ASCII block, you can now write data.

16. In the ASCII block, type in "A=0.20". Hit send. You should see steering turn.
 * A= is the format that software sends in data. We are simulating that. Steering can range from values of
 -0.25 to 0.25. Do not go beyond these values.

