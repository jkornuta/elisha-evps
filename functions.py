# Python functions that could be used in serialInterface.py

#********************************************#
# Function to test sending uint32 as 4 bytes #
#********************************************#
def send_long(ser):
  # Define value of uint32 to send
  x = 15000
  print "\n> Sending value: %s" % x 

  # Send serially
  val = struct.pack('I', x)
  ser.write(val)

  # What did it hear?
  while (1):
    # Stop/exit if user hits Ctrl-C
    try:
      # While data exists in the serial buffer
      while ser.inWaiting() > 0:
      # Yank line of data and print
        line = ser.readline()
        print "> Value received: %s" % line
    # Ctrl-C gracefully exits loop    
    except (KeyboardInterrupt, SystemExit):
      #ser.close()
      #print "\n> Serial closed. Exiting...\n"
      print "\n"
      prompt(ser)

#********************************************#
# Function to test sending float as 4 bytes  #
#********************************************#
def send_float(ser):
  # Define value of uint32 to send
  x = 1522.873
  print "\n> Sending value: %s" % x 

  # Send serially
  ser.write('f')  # Send 'f' character first
  val = struct.pack('f', x)
  ser.write(val)

  # What did it hear?
  while (1):
    # Stop/exit if user hits Ctrl-C
    try:
      # While data exists in the serial buffer
      while ser.inWaiting() > 0:
      # Yank line of data and print
        line = ser.readline()
        print "> Value received: %s" % line
    # Ctrl-C gracefully exits loop    
    except (KeyboardInterrupt, SystemExit):
      #ser.close()
      #print "\n> Serial closed. Exiting...\n"
      print "\n"
      prompt(ser)

