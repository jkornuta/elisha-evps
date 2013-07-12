"""
  Python program to interface serially with EVPS

  Written by Jeff Kornuta, 10/8/12
"""

import serial, time, sys, struct, scipy.io
from numpy import *

#*******************#
# Global variables  #
#*******************#
### Pull input data from *.mat file
path = '../../Spring 2013/'
#data = scipy.io.loadmat(path + 'System Inputs/mpcTest-20130306-1.mat')
#data = scipy.io.loadmat(path + 'System Inputs/idinput-20130325_sine.mat')
### Inputs stored in variable "u" when saved in MATLAB#
### For single input to system
#  u = data['u'].flatten().astype(float32) 
#  data_length = u.size
### For two inputs to system
#u = data['u'].astype(float32) 
### CUSTOM PYTHON DATA
t = arange(0,10,0.005)
u1 = 100*sin(2*pi*0.2*t)+50
 #u1 = Kv*u1 # uL/hr -> mm/s -> V, for a 10uL syringe
 #u1 = 0*t + 200
u2 = 0*t + 3.0
 #u2 = 0.5*sin(2*pi*0.5*t) + 3.5
u = array([u1, u2]).T
 #u = ones((1000,2))
u = u.astype(float32)
### Data length
data_length = u.size/2
#### Store input data as serial-compatible strings for sending
u1_string = [ ]
u2_string = [ ]
#### Adjust signal multiplier if desired
multiplier1 = 1.0 # Multiplier for signal 1 (since original is just 1 or -1)
multiplier2 = 1.0 # Multiplier for signal 2 (since original is just 1 or -1)
#multiplier1 = 1.0 
#multiplier2 = 1.0
#### Load up input values in string array
for i in xrange(data_length):
  u1_string.append( struct.pack('f', multiplier1*u[i,0]) )
  u2_string.append( struct.pack('f', multiplier2*u[i,1]) )


#**********************#
# Start main function  #
#**********************#
def main():
  # Create serial object
  BAUDRATE = 921600 # 115200 if slow
  #
  # ...For Windows
  #ser = serial.Serial()
  #ser.port = 11
  #ser.baudrate = BAUDRATE
  
  # ...For Mac OSX
  ser = serial.Serial('/dev/tty.usbserial-AE00DRYI', BAUDRATE)
  
  # Open serial connection
  ser.timeout = 5
  ser.open()
  ser.isOpen()

  # Wait initially for connection to establish
  print "Connecting to device...\n"
  time.sleep(7)

  # Print output/status from system
  while ser.inWaiting() > 0:
    line = ser.readline()
    sys.stdout.write(line)

  # Start prompt
  prompt(ser)
  
#**********************#
# Start the prompt     #
#**********************#
def prompt(ser):
  # Give summary of available commands to user
  print "Type 'start' to begin, 'send' to send data, or 'exit' to quit."

  # Wait for user to decide what to do.
  while (1):
    # Get user input
    input = raw_input(">> ")

    # Break if receive exit
    if input == 'exit':
      ser.close()
      exit()
    # Grab serial data if GO command is received
    elif input == 'start':
      ser.write('g')
      #mpc_serial(ser)
      go_serial(ser)
      #basic(ser)
      continue
    # Send size of data
    elif input == 'send':
      send_data(ser)
      continue
    # Otherwise, wait for another command
    else:
      print "Unrecognized command.\n"
      continue


#***************************************************#
# Function to continuously R/W serial data for MPC  #
#***************************************************#
def mpc_serial(ser):
  # Open file to store incoming serial data (file name is current time stamp)
  filename = time.strftime("data_%Y%m%d%H%M%S.txt")
  #filename = path + 'SysID Data/' + filename   # For SysID experiments
  filename = path + 'Experiments/' + filename   # For all other experiments
  file = open(filename,'w')
  print "> Grabbing and storing data...\n"
  """
  # Send first data to start timing off on a good foot
  ser.write( u1_string[0] )
  ser.write( u2_string[0] )
  """
  # Send first Hp+1 inputs to serial buffer
  Hp = 5  # Prediction horizon
  for i in xrange(Hp+1):
    ser.write( u1_string[i] )
    ser.write( u2_string[i] )

  # Start data counter (offset by data already sent)
  data_counter = Hp+1

  # Grab data!
  while (1):
    # Send, grab and store data, but stop/exit if user hits Ctrl-C
    try:
      # Read in line of data, print, and write to file
      line = ser.readline()
      # Once received a line of data, send next data immediately
      #   (twice to simulate to desired inputs)
      if data_counter < data_length:
        ser.write( u1_string[data_counter] )
        ser.write( u2_string[data_counter] )
      else:   # For MPC prediction horizon, just send last data point until done
        ser.write( u1_string[data_length-1] )
        ser.write( u2_string[data_length-1] )
      
      # Increment counter
      data_counter = data_counter + 1
  
      # Output data to screen and write to file
      sys.stdout.write(line)
      file.write(line)

      # Close if done
      if data_counter == data_length + (Hp):
        file.close()
        print "\n> Done. File closed. Exiting...\n"
        prompt(ser)

    # Ctrl-C gracefully exits loop    
    except (KeyboardInterrupt, SystemExit):
      file.close()
      #ser.close()
      print "\n> File closed. Exiting...\n"
      prompt(ser)
      #exit()


#*******************************************************#
# Function to continuously R/W serial data for ID/other #
#*******************************************************#
def go_serial(ser):
  # Open file to store incoming serial data (file name is current time stamp)
  filename = time.strftime("data_%Y%m%d%H%M%S.txt")
  #filename = path + 'SysID Data/' + filename   # For SysID experiments
  filename = path + 'Experiments/' + filename   # For all other experiments
  file = open(filename,'w')
  print "> Grabbing and storing data...\n"

  # Send first Hp+1 inputs to serial buffer
  Hp = 5  
  for i in xrange(Hp+1):
    ser.write( u1_string[i] )
    ser.write( u2_string[i] )

  # Start data counter (offset by data already sent)
  data_counter = Hp+1

  # Grab data!
  while (1):
    # Send, grab and store data, but stop/exit if user hits Ctrl-C
    try:
      # Read in line of data, print, and write to file
      line = ser.readline()
      # Once received a line of data, send next data immediately
      if data_counter < data_length:
        ser.write( u1_string[data_counter] )
        ser.write( u2_string[data_counter] )
                 
      # Increment counter
      data_counter = data_counter + 1
  
      # Output data to screen and write to file
      sys.stdout.write(line)
      file.write(line)

      # Close if done
      if data_counter == data_length + (Hp):
        file.close()
        print "\n> Done. File closed. Exiting...\n"
        prompt(ser)

    # Ctrl-C gracefully exits loop    
    except (KeyboardInterrupt, SystemExit):
      file.close()
      #ser.close()
      print "\n> File closed. Exiting...\n"
      prompt(ser)
      #exit()



######## BASIC FUNCTION FOR TESTING ############
def basic(ser):
  # Open file to store incoming serial data (file name is current time stamp)
  filename = time.strftime("data_%Y%m%d%H%M%S.txt")
  #filename = path + 'SysID Data/' + filename   # For SysID experiments
  filename = path + 'Experiments/' + filename   # For all other experiments
  file = open(filename,'w')
  print "> Grabbing and storing data...\n"
  # Grab data!
  while (1):
    # Send, grab and store data, but stop/exit if user hits Ctrl-C
    try:
      # Read in line of data, print, and write to file
      line = ser.readline()
        
      # Output data to screen and write to file
      sys.stdout.write(line)
      file.write(line)

    # Ctrl-C gracefully exits loop    
    except (KeyboardInterrupt, SystemExit):
      file.close()
      #ser.close()
      print "\n> File closed. Exiting...\n"
      prompt(ser)
      #exit()


#********************************************#
# Function to send input data to board       #
#********************************************#
def send_data(ser):
  """
  # TESTING
  print " Enter how many floats (1.0's) to send:"
  input = raw_input(" > ")
  u = ones( int(input) ).astype(float32)
  data_length = u.size
  """

  # Send $ to get this party started
  ser.write('$')
  
  # Now, send amount of data to be sent (string-packed)
  ser.write( struct.pack('I',data_length) )

  """
  # OK, now you can send the data (string-packed)
  #ser.write( u.tostring() ) # if already array of bytes
  #ser.write( struct.pack('%sf' % data_length, *u) )
  for i,value in enumerate(u):
    ser.write( struct.pack('f',value) )
    if i % 100 == 0:  # if a multiple of 100
      time.sleep(3e-3)
  """

  # What did it hear?
  while (1):
    # Stop/exit if user hits Ctrl-C
    try:
      # While data exists in the serial buffer
      while ser.inWaiting() > 0:
      # Yank line of data and print
        line = ser.readline()
        #sys.stdout.write(line)
        # Was data send successfully?
        if line == (str(data_length)+'\r\n'):
          print "> Data sent successfully.\n"
          prompt(ser)
        else:
          print "> Data transfer unsuccessful. Try again.\n"
          prompt(ser)
        #sys.stdout.write(line)
    # Ctrl-C gracefully exits loop    
    except (KeyboardInterrupt, SystemExit):
      #ser.close()
      #print "\n> Serial closed. Exiting...\n"
      print "\n"
      prompt(ser)


#*****************#
# Pythonic stuff  #
#*****************#

# Run main() if executed from command line
if __name__ == "__main__":
  main()


