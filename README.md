#cougballoon2.0
Code for the 2016 #cougballoon project

The code this year will be simplified as compared to last year. 

We will have a Teensy 3.2 microcontroller to serve as the main onboard processor. 

The GPS info is collected once a second via interrupt and stored in a global variable. It is connected to the Teensy via Hardware Serial for simplicity. No parsing of the data is done by the #cougballoon processor, the string itself will consist of NMEA 2.0 RMC and GGA sentences concatenated together with a newline character at the end of each. See http://www.gpsinformation.org/dale/nmea.htm for more information regarding NMEA sentences. 

The sensor data will be captured once a minute with the data being saved into variables. The data will be collected via ADC, concatenated with the other info to be transmitted, and sent in raw form. This will alleviate the #cougballoon processor from excessive work. The data will be translated on the server upon receipt in the receiving php script and then placed into the MySQL database.

The camera statuses will be captured and saved into a byte (1 = camera on, 0 = camera off). 

Once a minute when all of the data has been captured, a fresh copy of the GPS data will be retrieved and all data will be concatenated together into one string. That string will then be fed to the Iridium transmitter and sent to the satellite. 

The server side php script will handle the incoming HTTP POST data from the satellite service provider Rock7. Upon receipt of the HTTP POST command, the data will be parsed and placed into a MySQL database. 

When a user opens the http://www.cougballoon.com/launch2016.php page, the php script will retrieve the appropriate data from the MySQL database and send the HTML to the client to be displayed in the browser.
