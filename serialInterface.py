"""
  Python program to interface serially with EVPS

  Written by Jeff Kornuta, 10/8/12
"""

import serial, time, sys

#**********************#
# Start main function  #
#**********************#
def main():
  # Open serial connection
  ser = serial.Serial('/dev/tty.usbserial-AE00DRYI', 115200)
  ser.open()
  ser.isOpen()

  # Wait initially for connection to establish
  print "Opening serial port...\n"
  time.sleep(8)

  # Print output/status from system
  while ser.inWaiting() > 0:
    line = ser.readline()
    sys.stdout.write(line)

  # Give summary of available commands to user
  print "Type 'go' to begin or 'exit' to quit."

  # Wait for user to decide what to do.
  while (1):
    # Get user input
    input = raw_input(">> ")

    # Break if receive exit
    if input == 'exit':
      ser.close()
      exit()
    # Grab serial data if GO command is received
    elif input == 'go':
      ser.write('g\n')
      grab_serial(ser)
    # Otherwise, wait for another command
    else:
      print "Unrecognized command.\n"
      continue

#********************************************#
# Function to continuously grab serial data  #
#********************************************#
def grab_serial(ser):
  # Open file to store incoming serial data
  file = open('file.txt','w')
  print "> Grabbing and storing data...\n"

  # Continuously grab data
  while (1):
    # Grab and store data, but stop/exit if user hits Ctrl-C
    try:
      # While data exists in the serial buffer
      while ser.inWaiting() > 0:
        # Yank line of data, print, and write to file
        line = ser.readline()
        sys.stdout.write(line)
        file.write(line)
    # Ctrl-C gracefully exits loop    
    except (KeyboardInterrupt, SystemExit):
      file.close()
      ser.close()
      print "\n> File closed, serial closed. Exiting...\n"
      exit()


#*****************#
# Pythonic stuff  #
#*****************#

# Run main() if executed from command line
if __name__ == "__main__":
  main()


