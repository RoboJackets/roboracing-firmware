# How to architect UDP communications for MBED

## 3 thread architecture:
Leave NetworkInterface::connect() in blocking mode. We want comms threads to wait till connection established.

We open the network socket in blocking mode with a timeout (maybe 0.1 seconds?).
With blocking mode we can have a thread just wait till new data received. But in blocking mode
sending data can block for the timeout period. Se set blocking period short.

Use global variables for inter-thread communications.

### Thread 1: General computations (controls, etc).
 * Updates global variables as needed for communications 

### Thread 2: Sends repeating messages (ex. brake force target, current manual speed). Checks connection status
 * Checks get_connection_status(). See https://os.mbed.com/docs/mbed-os/v6.15/apis/network-interface.html
 * * If no connection closes + destroys network socket.
 * * If new connection established creates socket + sets it up.
 * Only sends data if connection good and socket exists.
 * Gets data to send from the global variables

### Thread 3: Waits for new data + processes it
 * Only does something if network socket exists.
 * With socket in blocking mode, recv() blocks till data received.
 * * NSAPI_ERROR_WOULD_BLOCK means read all the data
 * * Other negative numbers mean some sort of error.
 * Send immediate reply if message received needs a reply
 * Updates global variables as necessary.

### Problems:
 * How to deal with socket losing connection? What happens if recv() still blocking on destroy?
 * What happens if Thread 2 and Thread 3 try to send at the same time? Maybe need a mutex?
 
## 2 thread architecture:
Leave NetworkInterface::connect() in blocking mode. We want comms threads to wait till connection established.

### Thread 1: General computations (controls, etc).
 * Updates global variables as needed for communications 
 
### Thread 2: All communications
Gets data to send from global variables. Places data received in global variables.

Socket in blocking mode with short (<0.05s) timeout. 

 1. Checks get_connection_status(). See https://os.mbed.com/docs/mbed-os/v6.15/apis/network-interface.html
 * * If no connection closes + destroys network socket.
 * * Then calls connect()
 * * If new connection established creates socket + sets it up.
 2. Sends data if connection good and socket exists.
 * * If error other than WOULD_BLOCK, we lost connection. Destroy socket.
 3. If connection good and socket exists, Loop, calling socket.recv() till 0.1 sec elapses.
 * * If get WOULD_BLOCK message, everything OK. If get a different error code then destroy socket.
 * * If get message, put response into global variables.
 * * If message needs response, send response immediately.

