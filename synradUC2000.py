import serial
import matplotlib.pyplot as plt

'''
This program is inteded to control a Synrad laser using the UC2000 controller
connected to the computer with a USB-to-Serial converter.
(Or, I suppose an actual RS-232 COM port connection. 
 ...or, I guess, an Ethernet-to-serial converter. 
 ...basically anything that provides a real or virtual COM-port connection.)

The checksum on the UC2000 needs to be disabled. To do this:
    - Enter setup mode by pressing SELECT and ENTER at the same time
    - Press SELECT until the "CHECKSUM (ON)" text is selected.
    - Press ENTER so that the text reads "CHECKSUM", which corresponds to
      the situation where the checksum is not used.
      
THIS SCRIPT FIRES THE LASER BY DEFAULT. 
UNPLUG THE UC2000 FROM THE LASER BEFORE RUNNING THIS SCRIPT.
...or be ready for the laser to fire! :)

'''

##########################################
# change these parameters:
com_port = 3
power = 10  # percent
shot_time = 0.5  # seconds
delay     = 0.5  # time between shots (seconds)
num_shots = 2    # number of shots

###########################################

def main():
    print('Initializing UC2000')
    
    with serial.Serial('COM%i'%com_port, baudrate=9600, timeout=0.05) as uc2000:
    
        mess = Message('status_request', False, False).message_bytes
        uc2000.write(mess)
        m = uc2000.readline()
        print('The Synrad UC2000 reports this status:', m)

        print ('Setting the power to %.1f percent.'%power)
        
        # set the percentage
        mess = Message('percent', power, False).message_bytes
        uc2000.write(mess)
                
        for n in range(num_shots):
            if n>0:
                plt.pause(delay)
                
            # Start the laser!
            mess = Message('lase', True, False).message_bytes
            uc2000.write(mess)
            
            # Wait for the target to be destroyed
            plt.pause(shot_time)
            
            # Stop the laser
            mess = Message('lase', False, True).message_bytes
            uc2000.write(mess)
            
            # Step 4: profit
        

# the rest of this code is from: https://github.com/TobyBi/Synrad-UC2000/blob/main/uc2000.py

# =============================================================================
# Parameters for Message object
# =============================================================================

# Dict for converting between command name and byte
_UC2000_COMMAND_BYTES = {
    "pwm_freq": {
        5: 0x77,
        10: 0x78,
        20: 0x7a
    },
    "gate_logic": {
        "up": 0x7a,
        "down": 0x7b
    },
    "max_pwm": {
        95: 0x7c,
        99: 0x7d,
    },
    "lase_on_power_up": {
        True: 0x30,
        False: 0x31
    },
    "mode" : {
        "manual": 0x70,
        "anc": 0x71,
        "anv": 0x72,
        "man_closed": 0x73,
        "anv_closed": 0x74,
    },
    "lase": {
        True: 0x75,
        False: 0x76,
    },
}

class Message():
    """
    REMOTE Message sent to UC-2000 on REMOTE mode through RS-232 serial
    port.

    Parameters
    ----------
    command : {"pwm_freq", "gate_logic", "max_pwm", "lase_on_power_up", "mode", "lase", "percent", "status_request"}
        Command name, will be converted to command byte.
    data : float
        Data for PWM (or SET for closed loop) command.
    checksum : bool
        Checksum protocol mode.

    Notes
    -----
    There are 5 types of messages with different formats sent;
        Setup ("pwm_freq", "gate_logic", "max_pwm", "lase_on_power_up")
        Mode ("mode")
        PWM (or closed loop SET) ("percent")
        Lase ("lase")
        Status Request

    Setup Mode, and Lase commands have the byte sequence:
        STX<Command><Checksum>
        STX - start transmission byte.
        The checksum byte is the one's compliment of the Command byte.

    PWM (or SET) command byte sequence:
        STX<Command><Data Byte><Checksum>
        Data byte is the PWM percentage multipled by 2, converted into hex.
        The checksum byte is the adding without carry between command and
        data byte and then performing the one's compliment.

    Response from Setup, Mode, Lase, and PWM is either ACK (0xAA) or NAK
    (0x3F). A NAK is sent if there is no valid command or checksum byte
    sent within 1s of STX byte or if the checksum byte is wrong.

    Status Request:
        Single byte to tell UC-2000 to report it's status.

    Response from Status Request is
        ACK<Status Byte1><Status Byte2><PWM Byte><Power Byte><Checksum>
        Refer to the UC-2000 manual for futher details about the contents
        of the response bytes. Currently not using so not as important.

    TODO: include parsing response byte from UC-2000

    Examples
    --------
    >>> message = Message("percent", 10, False)
    >>> message.message_bytes()
    [126, 127, 20]

    Create message for setting PWM percent to 10%.

    >>> message = Message("lase", True, False)
    >>> message.message_bytes()
    [126, 127, 117]

    Create message for turning on command signal.
    """
    _start_byte = 0x5b
    # (STX) First byte sent to initialise communication, not needed when sending request.
    _status_request_byte = 0x7e
    _set_percent_byte = 0x7f

    def __init__(self, command: str, data, checksum: bool):
        """Inits a Message object."""
        self.command = command
        """Command to perform"""
        self.checksum = checksum
        """Checksum protocol mode."""
        self.data = data
        """Data for PWM (or SET for closed loop) command."""

    @property
    def message_bytes(self):
        """
        Creates and returns REMOTE message byte sequence.

        Returns
        -------
        message : list of int
            The message sequence containing the start byte, command byte,
            [data byte (optional)], and checksum (optional)
        """
        if self.command in _UC2000_COMMAND_BYTES.keys():
            command_byte = _UC2000_COMMAND_BYTES[self.command][self.data]

            message = [self._start_byte, command_byte]

            if self.checksum:
                # without data, the checksum is the one's compliment of the
                # command byte
                checksum_byte = ~command_byte & 0xff
                message.append(checksum_byte)

        elif self.command == "percent":
            try:
                message = [
                    self._start_byte, self._set_percent_byte, int(2*self.data)]
            except ValueError:
                raise ValueError(
            "Type of data is invalid. Needs to be float or int.")

            if self.checksum:
                # with data, the checksum is the addition without carry of the
                # command and data byte and then one's complimented
                checksum_byte = (
                    ~self.add_no_carry(
                        self._set_percent_byte, self.data) & 0xff
                    )
                message.append(checksum_byte)

        elif self.command == "status_request":
            message = [self._status_request_byte]

        else:
            raise ValueError("Command is not recognised by UC-2000")

        return message

    @staticmethod
    def add_no_carry(*args):
        """
        Addition without carry; addition is not carried to the next decimal up.

        Parameters
        ----------
        *args : iterable (not string)
            Iterate of ints to add without carry.

        Returns
        -------
        final_sum : int
            the result...

        Examples
        --------
        >>> add_no_carry(1, 1)
        2

        >>> add_no_carry(1, 18)
        19

        >>> add_no_carry(1, 19)
        10

        The '10' is not carried over to the next decimal.
        """
        num_digits = []

        for arg in args:
            num_digits.append(len(str(arg)))

        max_digits = max(num_digits)
        # list comprehension way
        # max_digits = max([len(str(arg)) for arg in args])
        final_sum = 0

        for pwr in range(1, max_digits + 1): # iterate through ea decimal
            result_no_carry = 0
            for arg in args:
                if len(str(arg)) >= pwr:
                    # modulus sets the current decimal as the most significant
                    # decimal
                    # floor div selects the most significant decimal
                    result_no_carry += arg % 10**pwr // 10**(pwr - 1)

            # list comprehension way
            # result_no_carry = sum([arg % 10**pwr // 10**(pwr - 1) for arg in args if len(str(arg)) >= pwr])

            # final_sum = str(result_no_carry % 10) + final_sum
            final_sum += result_no_carry % 10
        
        print(final_sum)
        return int(final_sum)



if __name__ == '__main__':
    main()
