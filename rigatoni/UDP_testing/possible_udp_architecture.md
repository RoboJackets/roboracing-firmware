# How to architect UDP communications for MBED

## 3 thread architecture:
Leave NetworkInterface::connect() in blocking mode. We want comms threads to wait till connection established.

We open the network socket in blocking mode without a timeout.
With blocking mode we can have a thread just wait till new data received.

Nobody checks connection status since with ethernet the connection
gets automatically recreated when destroyed.

Use global variables for inter-thread communications.

### Thread 1: General computations (controls, etc).
 * Updates global variables as needed for communications 

### Thread 2: Sends messages
 * Uses ThisThread::flags_wait_any_until to wait till thread flag OR until 100ms from last send
 * * If wait returns and flag set, send message flag corresponds to. This lets us respond to incoming messages at once.
 * * If wait returns and flags not set, send repeating messages.
 * * By using wait_until and setting it to time out 100ms from the last scheduled message send, can ensure scheduled messages are not blocked for too long.
 * Gets data to send from the global variables

### Thread 3: Waits for new data + processes it
 * Only does something if network socket exists.
 * With socket in blocking mode, recv() blocks till data received.
 * * Negative numbers mean some sort of error. But we ignore errors.
 * Updates global variables as necessary with new info.
 * If need to reply to a message set a thread flag on the sending thread.

### Problems:
 * How to deal with socket losing connection? What happens if sending and receiving at the same time?
 
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

