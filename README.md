This is the 2025 Birobot code experiment with Python.

We are building upon another Github repository:
https://github.com/epanov1602/CommandRevSwerve/

A large quantity of the code in this is made from them so most of the credit goes to "epanov1602"

--------

We also have the new maxswerve java code which is running with our minimal 8-motor robot.

This came from: https://github.com/REVrobotics/MAXSwerve-Java-Template

I just edited the CAN IDs to match our motors in: Constants.java

Other constants still need to be changed.

I had to disable CAN bus termination in the PDP as well.

The PDP appeares to have a firmware upgrade that caps the battery voltae display to 12.0 after a few seconds.
