"""
  Python program to interface serially with EVPS

  Written by Jeff Kornuta, 4/5/13
"""

import serial, time, sys, struct, scipy.io, os
from numpy import *

#************************************#
# Global variables and data loading  #
#************************************#
### Specify path of files
path = '../../Spring 2013/'

### For identification (mode 1), fill up appropriate array
data = scipy.io.loadmat(path + 'System Inputs/idinput-20130612_rbs300_30Hz_15sec.mat')
# Inputs stored in variable "u" when saved in MATLAB
u = data['u'].astype(float32)
# Data length
data_length_mode1 = u.size/2
# Initialize storage arrays
u1_string_mode1 = [ ]
u2_string_mode1 = [ ]
# Optional multipliers for signals 1 and 2
multiplier1 = 0.6
multiplier2 = 0.6
# Load up input values in string array
for i in xrange(data_length_mode1):
  u1_string_mode1.append( struct.pack('f', multiplier1*u[i,0]) )
  u2_string_mode1.append( struct.pack('f', multiplier2*u[i,1]) )


### For control (mode 2), fill up appropriate array
t = arange(0,10,3.333333333e-3)
u1 = sin(2*pi*2*t) + 3.0
u2 = sin(2*pi*1*t) - cos(2*pi*0.5*t) + 5.0
#u1 = 0*t + 2.0
#u2 = 0*t + 4.0
u = array([u1, u2]).T
u = u.astype(float32)
# Data length
data_length_mode2 = u.size/2
# Initialize storage arrays
u1_string_mode2 = [ ]
u2_string_mode2 = [ ]
# Optional multipliers for signals 1 and 2
multiplier1 = 1.0 
multiplier2 = 1.0 
# Load up input values in string array
for i in xrange(data_length_mode2):
  u1_string_mode2.append( struct.pack('f', multiplier1*u[i,0]) )
  u2_string_mode2.append( struct.pack('f', multiplier2*u[i,1]) )


#**********************#
# Start main function  #
#**********************#
def main():

  try:
    # Clear screen
    os.system( [ 'clear', 'cls' ][ os.name == 'nt' ] )

    # Create serial object
    BAUDRATE = 921600 # 115200 if slow
    #
    # ...For Windows
    #ser = serial.Serial()
    #ser.port = 11
    #ser.baudrate = BAUDRATE
  
    # ...For Mac OSX
    uno32 = serial.Serial('/dev/tty.usbserial-AE00DRYI', BAUDRATE)
  
    # Open serial connection
    uno32.timeout = 5
    uno32.open()
    uno32.isOpen()


    # Wait initially for connection to establish
    print "Connecting to device...\n"
    time.sleep(7)

    # Print output/status from system
    while uno32.inWaiting() > 0:
      line = uno32.readline()
      sys.stdout.write(line)

    # Start prompt
    prompt(uno32)
  
  except KeyboardInterrupt:
    print "Bye!"

  
#**********************#
# Start the prompt     #
#**********************#
def prompt(uno32):
  # Give summary of available commands to user
  print "Type 'help' to see list of commands or 'exit' to quit."

  # Wait for user to decide what to do.
  while (1):
    # Get user input
    input = raw_input(">> ")

    # Break if receive exit
    if input == 'exit':
      break
    # Start debug sequence
    elif input == 'debug':
      uno32.write('d')
      go_serial(uno32, 0.5) # Mode 0.5
      continue
     # Start debug sequence (write debug file)
    elif input == 'debugf':
      uno32.write('d')
      go_serial(uno32, 0) # Mode 0
      continue
    # Start identification
    elif input == 'ident':
      uno32.write('i')
      go_serial(uno32, 1) # Mode 1
      continue
    elif input == 'start':
      uno32.write('c')
      go_serial(uno32, 2) # Mode 2
      continue
    # Send size of data
    elif input == 'help':
      print """
      List of commands:
        'debug'  - Enter debug mode
        'debugf' - Enter debug mode (saving output to file)
        'ident'  - Start system identification
        'start'  - Start control experiment

      Life advice -- if in a bind, 'Ctrl+C'!
      """
      continue
    # Otherwise, wait for another command
    else:
      print "Unrecognized command.\n"
      continue

  # OK, out of while loop -- exit!
  uno32.close()
  os._exit(1)


#*******************************************************#
# Function to continuously R/W serial data for ID/other #
#*******************************************************#
def go_serial(uno32, mode):
  # DEBUG mode (mode 0 or 0.5)
  if mode == 0:
    # File to store incoming serial data (file name is current time stamp)
    filename = time.strftime("debug_%Y%m%d%H%M%S.txt")
    filename = path + 'Experiments/' + filename   # For all other experiments
    print "> Starting debug mode...\n"
  if mode == 0.5:
    filename = os.devnull
    print "> Starting debug mode...\n"  

  # Identification mode
  if mode == 1:
    # Send and verify length of data
    data_length = data_length_mode1
    check_data(uno32, data_length)
    # File to store incoming serial data (file name is current time stamp)
    filename = time.strftime("data_%Y%m%d%H%M%S.txt")
    filename = path + 'SysID Data/' + filename   # For SysID experiments
    # Set array variables
    u1_string = u1_string_mode1
    u2_string = u2_string_mode1
    print "> Starting identification...\n"

  # Control mode
  if mode == 2:
    # Send and verify length of data
    data_length = data_length_mode2
    check_data(uno32, data_length)
    # File to store incoming serial data (file name is current time stamp)
    filename = time.strftime("data_%Y%m%d%H%M%S.txt")
    filename = path + 'Experiments/' + filename   # For all other experiments
    # Send over PI gains
    #uno32.write( struct.pack('f', 0.0) ) # Kp gain 0.037
    #uno32.write( struct.pack('f', 0.0) ) # Ki gain 0.42
    # Set array variables
    u1_string = u1_string_mode2
    u2_string = u2_string_mode2
    print "> Starting...\n"
  
  # Open file
  file = open(filename,'w')

  # Send first Hp+1 inputs to serial buffer
  Hp = 5
  if mode == 0 or mode == 0.5: # Mode 0
    for i in xrange(Hp+1):
      uno32.write(  '\x00\x00\x00\x00' ) # Dummy 0.0
      uno32.write(  '\x00\x00\x00\x00' ) # Dummy 0.0
  else: # Modes 1 or 2
    for i in xrange(Hp+1):
      uno32.write( u1_string[i] )
      uno32.write( u2_string[i] )

  # Start data counter (offset by data already sent)
  data_counter = Hp+1

  # Grab data!
  while (1):
    # Send, grab and store data, but stop/exit if user hits Ctrl-C
    try:
      # Read in line of data, print, and write to file
      line = uno32.readline()
      # Once received a line of data, send next data immediately
      if mode == 0 or mode == 0.5: # Mode 0
        uno32.write(  '\x00\x00\x00\x00' ) # Dummy 0.0
        uno32.write(  '\x00\x00\x00\x00' ) # Dummy 0.0
      else: # Modes 1 or 2
        if data_counter < data_length:
          uno32.write( u1_string[data_counter] )
          uno32.write( u2_string[data_counter] )
        else:   # For MPC prediction horizon, just send last data point until done
          uno32.write( u1_string[data_length-1] )
          uno32.write( u2_string[data_length-1] )

                 
      # Increment counter
      data_counter = data_counter + 1
  
      # Output data to screen and write to file
      sys.stdout.write(line)
      file.write(line)

      # Close if done (modes 1 or 2)
      if mode == 1 or mode == 2:
        if data_counter == data_length + (Hp):
          file.close()
          print "\n> Done. File closed. Exiting...\n"
          # Return to prompt
          #prompt(uno32)
          # Exit program
          uno32.close()
          os._exit(1)

    # Ctrl-C gracefully exits loop    
    except (KeyboardInterrupt, SystemExit):
      file.close()
      print "\n> File closed. Exiting...\n"
      # Return to prompt
      #prompt(uno32)
      # Exit program
      uno32.close()
      os._exit(1)


#********************************************#
# Function to send input data to board       #
#********************************************#
def check_data(uno32, data_length):
  # Send amount of data to be sent (string-packed)
  uno32.write( struct.pack('I',data_length) )

  # Check data integrity
  line = uno32.readline()
  # Was data send successfully?
  if line == (str(data_length)+'\r\n'):
    print "> Data sent successfully.\n"
  else:
    print "> ERROR: data transfer unsuccessful. Try again.\n"
    prompt(uno32)


#*****************#
# Pythonic stuff  #
#*****************#

# Run main() if executed from command line
if __name__ == "__main__":
  main()


