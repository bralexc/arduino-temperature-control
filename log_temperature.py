import serial
from datetime import datetime

# use port COM3 with baud rate 9600 - these must match the values on the arduino serial monitor
ser = serial.Serial('COM3', 9600)

# filename is YYMMDD_HHMMSS_temperature.log
logfile = '%s_temperature.log' % datetime.now().strftime('%y%m%d_%H%M%S')

# open file for writing
with open(logfile, 'w') as f:

  # add current date/time to the beggining of the logfile
  f.write('[%s] - Starting temperature log\n' % datetime.now().strftime('%Y/%b/%d %H:%M:%S'))
  f.write('t [s], T [C], heater state, heater power [%]\n')
  
  # infinite loop
  while(True):
    try:
      # read line from arduino serial, strip whitespaces and add new line char
      temp = ser.readline().strip() + '\n'
      
      # print to screen, write to file buffer and then write to disk
      print temp,
      f.write(temp)
      f.flush()
      
    # should there be any problem, close the serial stream/communication
    except:
      ser.close()
	  