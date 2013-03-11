This is the ArdEx library by `blavery`, originally posted to
<http://www.raspberrypi.org/phpBB3/viewtopic.php?t=23309>.

From the forum:

> The code attached I call "ArdEx" - Arduino Hardware Expansion. It
> allows the rPi to set arduino pins as input or output. It sets outputs
> HIGH or LOW. It does digital read and analog read. Not very ambitious?
> Doesn't need to be. It's just extra I/O for the rPi, and it does that
> fine.
> 
> ArdEx is inspired by Gordon's DRC which controls the arduino from
> USB/Serial. ArdEx works over i2c. Slave mode i2c appears to have a bug
> that restricts "restart" operation, ie combination "receive a command
> / send data bytes". So separate transactions are implemented for send
> and receive. Check the sketch code for sample python calls from the
> rPi.

